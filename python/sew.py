""" Spline Error Weighting
##########################

This module implements Spline Error Weighting (SEW), introduced by [Ovren2018a]_.

SEW is a method to automatically set knot spacing, and estimate the spline fit error (approximation error),
using gyroscope and accelerometer data.

To estimate the knot spacings for a :py:class:`.SplitTrajectory` you would do the following::

    so3_dt, so3_var = knot_spacing_and_variance(gyro_data, gyro_timestamps, q_gyro)
    r3_dt, r3_var = knot_spacing_and_variance(acc_data, acc_timestamps, q_acc)
    trajectory = SplitTrajectory(r3_dt, so3_dt)

.. rubric:: References

.. [Ovren2018a]
   Ovrén, H. and Forssén, P-E.
   *Spline Error Weighting for Robust Visual-Inertial Fusion*
   In Proceedings of the IEEE on Computer Vision and Pattern Recognition (CVPR)
   June 2018

.. [Mihajlovic1999]
   Mihajlovic, Z.; Goluban, A.; and Zagar, M.
   *Frequency Domain Analysis of B-Spline Interpolation*
   In Proceedings of the IEEE International Symposium on Industrial Electronics
   July 1999

"""

import numpy as np
import scipy.optimize

import argparse

def bspline_interp_freq_func(w, dt=1.0):
    """Cubic B-spline interpolation frequency function

    Implements the frequency response function of a cubic B-spline according to [Mihajlovic1999]_

    Parameters
    ------------
    w : ndarray
        Angular frequencies in rad/s
    dt : float
        Spline knot spacing

    Returns
    -------------
    H : ndarray
        Frequency response function for frequencies `w`
    """
    sinc = lambda x: 1.0 if x == 0 else np.sin(np.pi*x)/(np.pi*x)
    sinc = np.vectorize(sinc)
    def H(w):
        a = 3 * sinc(w/(2*np.pi))**4
        b = 2 + np.cos(w)
        return a / b
    return dt * H(w * dt)


def spline_interpolation_response(freqs, dt):
    """Cubic B-spline interpolation frequency function

    Implements the frequency response function of a cubic B-spline according to [Mihajlovic1999]_

    Parameters
    ------------
    freqs : ndarray
        Frequencies in Hz
    dt : float
        Spline knot spacing

    Returns
    -------------
    H : ndarray
        Frequency response function for frequencies `freqs`
    """
    H = bspline_interp_freq_func(2*np.pi*freqs, dt)
    return H / H[0]


def signal_energy(spectrum):
    return np.sum(np.abs(spectrum)**2) / len(spectrum)


def find_max_quality_dt(quality_func, min_q, min_dt, max_dt, verbose=False):
    """Find the largest dt given a quality function

        quality_func : A function that takes a knot spacing and returns a quality measure
        min_q : (float) Minimum quality
        min_dt, max_dt: (float) bounds for dt
    """


    # Endpoint check
    dt = max_dt

    q = quality_func(dt)
    if q >= min_q:
        if verbose:
            print('Found at endpoint')
        return dt

    if verbose:
        print('Endpoint {:.1e} < {:.1e}'.format(q, min_q))

    # Backtrack
    step = max_dt * 0.5
    max_quality = 0
    max_quality_dt = None
    while True:
        dt -= step

        dt = max(dt, min_dt)

        q = quality_func(dt)

        if verbose:
            print('Trying {:.4f}, q={:.3e}'.format(dt, q))

        if q > min_q:

            root_func = lambda dt: quality_func(dt) - min_q
            if verbose:
                print('{:.1e} > {:.1e}: Switching to Brent'.format(q, min_q))
                print(root_func(dt), root_func(max_dt))
            brent_dt = scipy.optimize.brentq(root_func, dt, max_dt)
            if verbose:
                print('Found dt={:.3e}'.format(brent_dt))
            return brent_dt
        else:
            step *= 0.5
            if q > max_quality:
                max_quality = q
                max_quality_dt = dt
    
            if dt <= min_dt:
                if verbose:
                    print('dt too small: No dt satisfies condition. Returning best: {:.1e}'.format(max_quality_dt))
                return max_quality_dt                


def find_uniform_knot_spacing_spectrum(Xhat, times, quality, *, verbose=False, min_dt=None, max_dt=None):
    "Find the uniform knot spacing that keeps quality percent signal energy"
    
    sample_rate = 1 / np.mean(np.diff(times))
    freqs = np.fft.fftfreq(len(times), d=1/sample_rate)
    max_remove = signal_energy(Xhat) * (1 - quality)

    def quality_func(dt):
        H = spline_interpolation_response(freqs, dt)
        removed = signal_energy((1 - H) * Xhat)
        return max_remove / removed

    if min_dt is None:
        min_dt = 1/sample_rate

    if max_dt is None:
        max_dt = (len(times) / 4) / sample_rate

    return find_max_quality_dt(quality_func, 1.0, min_dt, max_dt, verbose=verbose)
    

def find_uniform_knot_spacing(signal, times, quality, *, verbose=False):
    "Find the uniform knot spacing that keeps quality percent signal energy"
    
    Xhat = make_reference_spectrum(signal)
    return find_uniform_knot_spacing_spectrum(Xhat, times, quality, verbose=verbose)


def make_reference_spectrum(signal):
    if signal.ndim == 1:
        signal = np.atleast_2d(signal)
    elif not signal.ndim == 2:
        raise ValueError("Signal must be at most 2D")
        
    d, _ = signal.shape
    S = np.fft.fft(signal, axis=1)
    S[:, 0] = 0 # Remove DC component
    Xhat = np.sqrt(1/d) * np.linalg.norm(S, axis=0)
    return Xhat


def quality_to_variance(signal, q):
    return (1-q) * np.mean(ref**2)


def quality_to_variance_spectrum(spectrum, q):
    N = len(spectrum)
    return (1-q) * np.mean(spectrum**2) / N


def dt_to_variance_spectrum(spectrum, freqs, spline_dt):
    H = spline_interpolation_response(freqs, spline_dt)
    EE = signal_energy((1-H) * spectrum)
    return EE / len(spectrum)
    
    
def knot_spacing_and_variance(signal, times, quality, min_dt=None, max_dt=None, verbose=False):
    """Find knot spacing and variance from signal

    Given a quality value, this function first determines the maximum knot spacing
    that achieves this quality, given the data in signal.
    It then computes the estimated variance of the spline fit error, using this knot spacing.

    Parameters
    ------------
    signal : ndarray
            Signal to fit to (e.g. accelerometer or gyroscope)
    times : ndarray
            Timestamps of signal
    quality : float
            Requested quality, in the range 0-1. A good value is usually around 0.99.
    min_dt : float
            Lowest acceptable value of knot spacing
    max_dt : float
            Highest acceptable value of knot spacing
    verbose: bool
            Print search steps

    Returns
    -------------
    dt : float
        The found knot spacing
    var : float
        The spline fit error for this signal and knot spacing
    """
    Xhat = make_reference_spectrum(signal)
    dt = find_uniform_knot_spacing_spectrum(Xhat, times, quality, min_dt=min_dt, max_dt=max_dt, verbose=verbose)
    sample_rate = 1 / np.mean(np.diff(times))
    freqs = np.fft.fftfreq(len(Xhat), d=1/sample_rate)
    variance = dt_to_variance_spectrum(Xhat, freqs, dt)
    return dt, variance

