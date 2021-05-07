import numpy as np
#import ipdb

def fidelite(x):
    J = []
    taille = 10
    for i in range(taille):
        J.append(calcul_rms(x[i*100:i*100+100]))
    Vmin = np.min(J)
    Vmax = np.max(J)

    return Vmax-Vmin

def variance_rms(x):
    J = []
    taille = 10
    for i in range(taille):
        J.append(calcul_rms(x[i*100:i*100+100]))
    return np.sqrt(np.var(J))

def calcul_rms(x, axis=None):
        return np.sqrt(np.mean(x**2, axis=axis))

def calcul_offset(vect_x, vect_y, vect_z, taille_cal_offset):
    taille_tab = len(vect_x)
    acc_x = acc_y = acc_z  = 0
    for i in  range(taille_cal_offset):
        acc_x += vect_x[i]
        acc_y += vect_y[i]
        acc_z += vect_z[i]
    offset_x = acc_x / taille_cal_offset
    offset_y = acc_y / taille_cal_offset
    offset_z = acc_z / taille_cal_offset

    offset = np.vstack((offset_x, offset_y, offset_z))
    return offset

def double_integration(vect_x, vect_y, vect_z, velocity_init, position_init, delta_t):
    taille_vect = len(vect_x)
    velocity_x = []
    velocity_y = []
    velocity_z = []

    position_x = []
    position_y = []
    position_z = []
    for i in range(taille_vect):
        if (i == 1):
            #Attention a le faire plus proprement pour initialisation different de
            # X(0, 0,0) V(0, 0, 0)
            velocity_x.append(velocity_init[0])
            velocity_y.append(velocity_init[1])
            velocity_z.append(velocity_init[2])

            position_x.append(position_init[0])
            position_y.append(position_init[1])
            position_z.append(position_init[2])
        if (i == 2):
            velocity_x.append(velocity_init[0]+integration_trapeze_init(velocity_init[0], i, vect_x, delta_t[i]))
            velocity_y.append(velocity_init[1]+integration_trapeze_init(velocity_init[1], i, vect_y, delta_t[i]))
            velocity_z.append(velocity_init[2]+integration_trapeze_init(velocity_init[2], i, vect_z, delta_t[i]))

            position_x.append(position_init[0]+integration_trapeze_init(position_init[0], i, velocity_x, delta_t[i]))
            position_y.append(position_init[1]+integration_trapeze_init(position_init[1], i, velocity_y, delta_t[i]))
            position_z.append(position_init[2]+integration_trapeze_init(position_init[2], i, velocity_z, delta_t[i]))
        if (i > 2):
            #ipdb.set_trace()
            velocity_x.append(velocity_x[i-1]+integration_trapeze(i-1, i, vect_x, delta_t[i]))
            velocity_y.append(velocity_y[i-1]+integration_trapeze(i-1, i, vect_y, delta_t[i]))
            velocity_z.append(velocity_z[i-1]+integration_trapeze(i-1, i, vect_z, delta_t[i]))

            position_x.append(position_x[i-1]+integration_trapeze(i-1, i, velocity_x, delta_t[i]))
            position_y.append(position_y[i-1]+integration_trapeze(i-1, i, velocity_y, delta_t[i]))
            position_z.append(position_z[i-1]+integration_trapeze(i-1, i, velocity_z, delta_t[i]))
    #ipdb.set_trace()
    #    print np.shape(position_x)
    #    print np.shape(position_y)
    velocity = np.array([velocity_x, velocity_y, velocity_z])
    position = np.array([position_x, position_y, position_z])
    return velocity, position

def integration_trapeze(a, b, f, delta_t):
    f_a = f[a]
    f_b = f[b]
    val_int = (f_a+f_b)/2.
    val_int *= delta_t

    return val_int

def integration_trapeze_init(val_init, b, f, delta_t):
    f_a = val_init
    f_b = f[b]
    val_int = (f_a+f_b)/2.
    val_int *= delta_t

    return val_int

def position_dist_origine_planxy(x, y, x0=0, y0=0):
    x -= x0
    y -= y0
    res = np.sqrt(
        np.power(x, 2)
        +
        np.power(y, 2)
    )

    return res

def trouve_ind_divergence(vect, val_divergence):
    taille_vect = len(vect)
    for i  in range(taille_vect):
        if (vect[i] >= val_divergence):
            return i

def trouve_dis_div_temps(vect, temps, freq_ech):
    taille_vect = len(vect)
    print("taille vecteur : " +  str(taille_vect))
    print("temps : " + str(temps))

    nb_ech_stop = temps / (1/freq_ech)
    
    return vect[int(nb_ech_stop)]


def double_integration_simpson(vect_x, vect_y, vect_z, velocity_init, position_init, delta_t, nb_ech):
    taille_vect = len(vect_x)
    velocity_x = []
    velocity_y = []
    velocity_z = []

    position_x = []
    position_y = []
    position_z = []
    for i in range(taille_vect-nb_ech):
        if (i == 1):
            velocity_x.append(velocity_init[0])
            velocity_y.append(velocity_init[1])
            velocity_z.append(velocity_init[2])

            position_x.append(position_init[0])
            position_y.append(position_init[1])
            position_z.append(position_init[2])
        if (i == 2):
            velocity_x.append(velocity_init[0]+integration_simpson_init(velocity_init[0], i, vect_x, delta_t[i-1], nb_ech))
            velocity_y.append(velocity_init[1]+integration_simpson_init(velocity_init[1], i, vect_y, delta_t[i-1], nb_ech))
            velocity_z.append(velocity_init[2]+integration_simpson_init(velocity_init[2], i, vect_z, delta_t[i-1], nb_ech))

            position_x.append(position_init[0]+integration_simpson_init(position_init[0], i-1, velocity_x, delta_t[i-1], nb_ech))
            position_y.append(position_init[1]+integration_simpson_init(position_init[1], i-1, velocity_y, delta_t[i-1], nb_ech))
            position_z.append(position_init[2]+integration_simpson_init(position_init[2], i-1, velocity_z, delta_t[i-1], nb_ech))
        if (i > 2):
            #ipdb.set_trace()
            #print(i)
            #print("velocity : " + str(len(velocity_x)))
            velocity_x.append(velocity_x[i-2]+integration_simpson(i-1, i, vect_x, delta_t[i-1], nb_ech))
            velocity_y.append(velocity_y[i-2]+integration_simpson(i-1, i, vect_y, delta_t[i-1], nb_ech))
            velocity_z.append(velocity_z[i-2]+integration_simpson(i-1, i, vect_z, delta_t[i-1], nb_ech))
            #print(position_x[i-2])
            #print(velocity_x[i-1])
            #print(velocity_x[i-2])
            position_x.append(position_x[i-2]+integration_simpson(i-2, i-1, velocity_x, delta_t[i-1], nb_ech))
            position_y.append(position_y[i-2]+integration_simpson(i-2, i-1, velocity_y, delta_t[i-1], nb_ech))
            position_z.append(position_z[i-2]+integration_simpson(i-2, i-1, velocity_z, delta_t[i-1], nb_ech))
    #ipdb.set_trace()
    #    print np.shape(position_x)
    #    print np.shape(position_y)
    velocity = np.array([velocity_x, velocity_y, velocity_z])
    position = np.array([position_x, position_y, position_z])
    return velocity, position

def integration_simpson(a, b, f, delta_t, nb_ech):
    h = (b-a)/np.double(nb_ech)
    z = np.double(f[a]+f[b])

    for i in range(1,nb_ech,2) :
        z += 4 * f[np.int(a+i*h)]
    for i in range(2, nb_ech-1, 2):
        z += 4 * f[np.int(a+i*h)]

    val_int =  z*h /3
    val_int *= delta_t

    return val_int

def integration_simpson_init(val_init, b, f, delta_t, nb_ech):
    a = 0 # attention dans un cas réel
    f_a = val_init
    h = (b-a)/np.double(nb_ech)
    z = np.double(f_a+f[b])/6.
    for i in range(1,nb_ech) :
        z = z+f[np.int(a+i*h)]/3.
    for i in range(nb_ech) :
        z=z+f[np.int(a+(2.*i+1)*h/2)]*2./3.

    val_int =z*h
    val_int *= delta_t

    return val_int

def double_int_class(vect, delta_t, x0, v0):
    """calcul de la double integration à la bourrin

    :vect: TODO
    :delta_t: TODO
    :x0: TODO
    :y0: TODO
    :returns: TODO

    """
    x = []
    v = []

    for i in range(len(vect) - 1):
        if i==0 :
            v.append(delta_t[i] * vect[i] + v0)
        else:
            v.append(delta_t[i] * vect[i] + v[i-1])
    for i in range(len(vect) - 1):
        if i==0:
            x.append(delta_t[i] * v[i] + x0)
        else:
            x.append(delta_t[i] * v[i] + x[i-1])
    
    return np.array(x), np.array(v)