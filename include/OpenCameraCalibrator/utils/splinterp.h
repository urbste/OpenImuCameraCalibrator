// -- splinterp.h --
// Created by AJ Pryor on 2/2/17.
// C-style array support for 2D interp added by DL Elliott 09/2019

#ifndef SPLINTERP_H
#define SPLINTERP_H


#include <cmath>
#include <thread>
#include <vector>

#ifndef NUM_THREADS
#define NUM_THREADS 6
#endif
#ifndef SERIAL_LIMIT
#define SERIAL_LIMIT 4093
#endif
// Interpolate position x,y from 2D array stored in data, which is assumed to lie on an integer coordinate system.
// Values of x and y that lie outside of the dimensions of data are set to 0.
namespace splinterp{
    
    template <typename T>
    using func1D = void(*)(const T* const, 
                           const size_t&,
                           const T* const,
                           const size_t&, T*,
                           const long long&);
    
    template <typename T>
    void parallel_interp1(func1D<T> f, 
                          const T* const data, 
                          const size_t& nrows,
                          const T* const x,
                          const size_t& N, 
                          T* result,
                          const long long& origin_offset){

        if ( N <= SERIAL_LIMIT){ // for small numbers of elements just use 1 thread
            f(data,nrows,x,N,result,origin_offset);
        }
        else{ // launch multiple threads
            unsigned long long THREAD_CHUNK_SIZE = N / NUM_THREADS;

            std::vector<std::thread> workers;
            workers.reserve(NUM_THREADS);
            const T *tmp_x = x;
            T *tmp_result = result;
            int i;
            for ( i = 0; i < NUM_THREADS; ++i){
                
                if ( i == (NUM_THREADS-1) ){
                    workers.emplace_back(std::thread(f,data,nrows,tmp_x,(size_t)(x+N-tmp_x),tmp_result,origin_offset));
                    break;
                }
                else{
                    workers.emplace_back( std::thread(f,data,nrows,tmp_x,THREAD_CHUNK_SIZE,tmp_result,origin_offset) );
                }

                tmp_x+=THREAD_CHUNK_SIZE;
                tmp_result+=THREAD_CHUNK_SIZE;
            }
            for (auto& t:workers)t.join();
        }
    
    };
     
    template <typename T>
    using func1D_cx = void(*)(const T* const, const T* const,
                              const size_t&,
                              const T* const,
                              const size_t&, T*, T*,
                              const long long&);

    template <typename T>
    void parallel_interp1_cx(func1D_cx<T> f, 
                             const T* const data_r, 
                             const T* const data_i,
                             const size_t& nrows,
                             const T* const x,
                             const size_t& N, 
                             T* result_r,
                             T* result_i,
                             const long long& origin_offset){

        if ( N <= SERIAL_LIMIT){// for small numbers of elements just use 1 thread
            f(data_r,data_i,nrows,x,N,result_r,result_i,origin_offset);
        }
        else{// launch multiple threads
            unsigned long long THREAD_CHUNK_SIZE = N / NUM_THREADS;

            std::vector<std::thread> workers;
            workers.reserve(NUM_THREADS);
            const T *tmp_x = x;
            T *tmp_result_r = result_r;
            T *tmp_result_i = result_i;
            int i;
            for ( i = 0; i < NUM_THREADS; ++i){    
                
                if ( i == (NUM_THREADS - 1) ){
                    workers.emplace_back(std::thread(f,data_r,data_i,nrows,tmp_x,(size_t)(x+N-tmp_x),tmp_result_r,tmp_result_i,origin_offset));
                    break;
                }
                else{
                    workers.emplace_back( std::thread(f,data_r,data_i,nrows,tmp_x,THREAD_CHUNK_SIZE,tmp_result_r,tmp_result_i,origin_offset) );
                }

                tmp_x+=THREAD_CHUNK_SIZE;
                tmp_result_r+=THREAD_CHUNK_SIZE;
                tmp_result_i+=THREAD_CHUNK_SIZE;
            }
            for (auto& t:workers)t.join();
        }
    
    };
    
  
    template <typename T>
    using func2D = void(*)(const T* const, 
                           const size_t&,
                           const size_t&,
                           const T* const,
                           const T* const,
                           const size_t&, T*,
                           const long long&);
    
    template <typename T>
    void parallel_interp2(func2D<T> f, 
                          const T* const data, 
                          const size_t& nrows,
                          const size_t& ncols,
                          const T* const x,
                          const T* const y,
                          const size_t& N, 
                          T* result,
                          const long long& origin_offset){

        if ( N <= SERIAL_LIMIT){ // for small numbers of elements just use 1 thread
            f(data,nrows,ncols,x,y,N,result,origin_offset);
        }
        else{ // launch multiple threads
            unsigned long long THREAD_CHUNK_SIZE = N / NUM_THREADS;

            std::vector<std::thread> workers;
            workers.reserve(NUM_THREADS);
            const T *tmp_x = x;
            const T *tmp_y = y;
            T *tmp_result = result;
            int i;
            for ( i = 0; i < NUM_THREADS; ++i){

                if ( i == (NUM_THREADS - 1) ){
                    workers.emplace_back(std::thread(f,data,nrows,ncols,tmp_x,tmp_y,(size_t)(x+N-tmp_x),tmp_result,origin_offset));
                    break;
                }
                else{
                    workers.emplace_back( std::thread(f,data,nrows,ncols,tmp_x,tmp_y,THREAD_CHUNK_SIZE,tmp_result,origin_offset) );
                }
                
                tmp_x+=THREAD_CHUNK_SIZE;
                tmp_y+=THREAD_CHUNK_SIZE;
                tmp_result+=THREAD_CHUNK_SIZE;
            }
            for (auto& t:workers)t.join();
        }
    };
    
    
    
    template <typename T>
    using func2D_cx = void(*)(const T* const, const T* const,
                              const size_t&,
                              const size_t&,
                              const T* const,
                              const T* const,
                              const size_t&, T*, T*,
                              const long long&);

    template <typename T>
    void parallel_interp2_cx(func2D_cx<T> f, 
                             const T* const data_r, 
                             const T* const data_i,
                             const size_t& nrows,
                             const size_t& ncols,
                             const T* const x,
                             const T* const y,
                             const size_t& N, 
                             T* result_r,
                             T* result_i,
                             const long long& origin_offset){

        if ( N <= SERIAL_LIMIT){// for small numbers of elements just use 1 thread
            f(data_r,data_i,nrows,ncols,x,y,N,result_r,result_i,origin_offset);

        }
        else{// launch multiple threads
            unsigned long long THREAD_CHUNK_SIZE = N / NUM_THREADS;
            std::vector<std::thread> workers;
            workers.reserve(NUM_THREADS);
            const T *tmp_x = x;
            const T *tmp_y = y;
            T *tmp_result_r = result_r;
            T *tmp_result_i = result_i;
            int i;
            for ( i = 0; i < NUM_THREADS; ++i){

                if ( i == (NUM_THREADS - 1) ){
                    workers.emplace_back(std::thread(f,data_r,data_i,nrows,ncols,tmp_x,tmp_y,(size_t)(x+N-tmp_x),tmp_result_r,tmp_result_i,origin_offset));
                    break;
                }
                else{
                    workers.emplace_back( std::thread(f,data_r,data_i,nrows,ncols,tmp_x,tmp_y,THREAD_CHUNK_SIZE,tmp_result_r,tmp_result_i,origin_offset) );
                }
                
                tmp_x+=THREAD_CHUNK_SIZE;
                tmp_y+=THREAD_CHUNK_SIZE;
                tmp_result_r+=THREAD_CHUNK_SIZE;
                tmp_result_i+=THREAD_CHUNK_SIZE;
            }
            for (auto& t:workers)t.join();
        }
    
    };
    

    template <typename T>
    using func3D = void(*)(const T* const, 
                           const size_t&,
                           const size_t&,
                           const size_t&,
                           const T* const,
                           const T* const,
                           const T* const,
                           const size_t&, T*,
                           const long long&);
    
    template <typename T>
    void parallel_interp3(func3D<T> f, 
                          const T* const data, 
                          const size_t& nrows,
                          const size_t& ncols,
                          const size_t& nlayers,
                          const T* const x,
                          const T* const y,
                          const T* const z,
                          const size_t& N, 
                          T* result,
                          const long long& origin_offset){

        if ( N <= SERIAL_LIMIT){ // for small numbers of elements just use 1 thread
            f(data,nrows,ncols,nlayers,x,y,z,N,result,origin_offset);
        }
        else{ // launch multiple threads
            unsigned long long THREAD_CHUNK_SIZE = N / NUM_THREADS;

            std::vector<std::thread> workers;
            workers.reserve(NUM_THREADS);
            const T *tmp_x = x;
            const T *tmp_y = y;
            const T *tmp_z = z;
            T *tmp_result = result;
            int i;
            for ( i = 0; i < NUM_THREADS; ++i){

                if ( i == (NUM_THREADS - 1) ){
                    workers.emplace_back(std::thread(f,data,nrows,ncols,nlayers,tmp_x,tmp_y,tmp_z,(size_t)(x+N-tmp_x),tmp_result,origin_offset));
                    break;
                }
                else{
                    workers.emplace_back( std::thread(f,data,nrows,ncols,nlayers,tmp_x,tmp_y,tmp_z,THREAD_CHUNK_SIZE,tmp_result,origin_offset) );
                }

                tmp_x+=THREAD_CHUNK_SIZE;
                tmp_y+=THREAD_CHUNK_SIZE;
                tmp_z+=THREAD_CHUNK_SIZE;
                tmp_result+=THREAD_CHUNK_SIZE;
            }
            for (auto& t:workers)t.join();
        }
    
    };
    
    
    template <typename T>
    using func3D_cx = void(*)(const T* const, const T* const,
                              const size_t&,
                              const size_t&,
                              const size_t&,
                              const T* const,
                              const T* const,
                              const T* const,
                              const size_t&, T*, T*,
                              const long long&);

    template <typename T>
    void parallel_interp3_cx(func3D_cx<T> f, 
                             const T* const data_r, 
                             const T* const data_i,
                             const size_t& nrows,
                             const size_t& ncols,
                             const size_t& nlayers,
                             const T* const x,
                             const T* const y,
                             const T* const z,
                             const size_t& N, 
                             T* result_r,
                             T* result_i,
                             const long long& origin_offset){

        if ( N <= SERIAL_LIMIT){// for small numbers of elements just use 1 thread
            f(data_r,data_i,nrows,ncols,nlayers,x,y,z,N,result_r,result_i,origin_offset);
        }
        else{// launch multiple threads
            unsigned long long THREAD_CHUNK_SIZE = N / NUM_THREADS;

            std::vector<std::thread> workers;
            workers.reserve(NUM_THREADS);
            const T *tmp_x = x;
            const T *tmp_y = y;
            const T *tmp_z = z;
            T *tmp_result_r = result_r;
            T *tmp_result_i = result_i;
            int i;
            for ( i = 0; i < NUM_THREADS; ++i){

                if ( i == (NUM_THREADS - 1) ){
                    workers.emplace_back(std::thread(f,data_r,data_i,nrows,ncols,nlayers,tmp_x,tmp_y,tmp_z,(size_t)(x+N-tmp_x),tmp_result_r,tmp_result_i,origin_offset));
                    break;
                }
                else{
                workers.emplace_back( std::thread(f,data_r,data_i,nrows,ncols,nlayers,tmp_x,tmp_y,tmp_z,THREAD_CHUNK_SIZE,tmp_result_r,tmp_result_i,origin_offset) );

                }

                tmp_x+=THREAD_CHUNK_SIZE;
                tmp_y+=THREAD_CHUNK_SIZE;
                tmp_z+=THREAD_CHUNK_SIZE;
                tmp_result_r+=THREAD_CHUNK_SIZE;
                tmp_result_i+=THREAD_CHUNK_SIZE;
            }
            for (auto& t:workers)t.join();
        }
    
    };
    
    
    template <typename T>
    void interp1_F(const T* const data, 
                   const size_t& nrows,
                   const T* const x,
                   const size_t& N, T* result,
                   const long long& origin_offset=0){

        for (auto i = 0; i < N; ++i) { 
           // get coordinates of bounding grid locations
           long long x_1 = (long long) std::floor(x[i]) - origin_offset;
           
           // handle special case where x is the last element
           if ( (x[i] - origin_offset) == (nrows-1)){;x_1 -= 1;}
           // return 0 for target values that are out of bounds
              if (x_1 < 0 | (x_1+1) > (nrows - 1)){
                result[i] = 0; 
            } 
            else {
                // get the array values
               const T& f_1 = data[x_1];
               const T& f_2 = data[x_1+1];
                // compute weights
               T w_x1 = x_1+1 - (x[i]-origin_offset);
               result[i] = f_1 * w_x1 +f_2 - f_2*w_x1;

            }
        }
    }

    template <typename T>
    void interp1_F_cx(const T* const data_r, const T* const data_i,
                      const size_t& nrows, const T* const x, 
                      const size_t& N, T* result_r, T* result_i,
                      const long long& origin_offset=0){

        for (auto i = 0; i < N; ++i) { 
           // get coordinates of bounding grid locations
           long long x_1 = ( long long) std::floor(x[i]) - origin_offset;
           // handle special case where x is the last element
           if ( (x[i] - origin_offset) == (nrows-1)){x_1 -= 1;}
           // return 0 for target values that are out of bounds
              if (x_1 < 0 | (x_1+1) > (nrows - 1)){
                result_r[i] = 0;
                result_i[i] = 0;
            } 
            else {
                // get the array values
               const T& f_1_r = data_r[x_1];
               const T& f_2_r = data_r[x_1+1];
                // compute weights
               T w_x1 = x_1+1 - (x[i] - origin_offset);
               result_r[i] = f_1_r * w_x1 +f_2_r - f_2_r*w_x1;
               
               const T& f_1_i = data_i[x_1];
               const T& f_2_i = data_i[x_1+1];
               result_i[i] = f_1_i * w_x1 +f_2_i - f_2_i*w_x1;

            }
        }
    }
    
    template <typename T>
    void interp2_F(const T* const data,
                   const size_t& nrows, const size_t& ncols,
                   const T* const x, const T* const y,
                   const size_t& N, T* result,
                   const long long& origin_offset=0){

        for (auto i = 0; i < N; ++i) {

           // get coordinates of bounding grid locations
           long long x_1 = ( long long) std::floor(x[i]) - origin_offset;
           long long x_2 = x_1 + 1;
           long long y_1 = ( long long) std::floor(y[i]) - origin_offset;
           long long y_2 = y_1 + 1;

           // handle special case where x/y is the last element
           if ( (x[i] - origin_offset) == (nrows-1) )   { x_2 -= 1; x_1 -= 1;}
           if ( (y[i] - origin_offset) == (ncols-1) )   { y_2 -= 1; y_1 -= 1;}

           // return 0 for target values that are out of bounds
           if (x_1 < 0 | x_2 > (nrows - 1) |  y_1 < 0 | y_2 > (ncols - 1)){
                result[i] = 0;

            } 
            else {
                
                // get the array values
                const T& f_11 = data[x_1 + y_1*nrows];
                const T& f_12 = data[x_1 + y_2*nrows];
                const T& f_21 = data[x_2 + y_1*nrows];
                const T& f_22 = data[x_2 + y_2*nrows];

                // compute weights
                T w_x1 = x_2 - (x[i] - origin_offset);
                T w_x2 = (x[i] - origin_offset) - x_1;
                T w_y1 = y_2 - (y[i] - origin_offset);
                T w_y2 = (y[i] - origin_offset) - y_1;

                T a,b;
                a = f_11 * w_x1 + f_21 * w_x2;
                b = f_12 * w_x1 + f_22 * w_x2;
                result[i] = a * w_y1 + b * w_y2;
            }
        }
    }

	// NOTE: x and y are swapped in C version.  x is columns, y is rows
	// NOTE: for best performance, provide x,y arrays of positions in row-major order
	template <typename T>
	void interp2_C(const T* const data,
		const size_t& nrows, const size_t& ncols,
		const T* const x, const T* const y,
		const size_t& N, T* result,
		const long long& origin_offset = 0) {

		for (auto i = 0; i < N; ++i) {

			// get coordinates of bounding grid locations
			long long x_1 = (long long)std::floor(x[i]) - origin_offset;
			long long x_2 = x_1 + 1;
			long long y_1 = (long long)std::floor(y[i]) - origin_offset;
			long long y_2 = y_1 + 1;

			// handle special case where x/y is the last element
			if ((x[i] - origin_offset) == (ncols - 1)) { x_2 -= 1; x_1 -= 1; }
			if ((y[i] - origin_offset) == (nrows - 1)) { y_2 -= 1; y_1 -= 1; }

			// return 0 for target values that are out of bounds
			if (x_1 < 0 | x_2 > (ncols - 1) | y_1 < 0 | y_2 > (nrows - 1)) {
				result[i] = 0;

			}
			else {

				// get the array values
				const T& f_11 = data[x_1 + y_1 * ncols];
				const T& f_12 = data[x_1 + y_2 * ncols];
				const T& f_21 = data[x_2 + y_1 * ncols];
				const T& f_22 = data[x_2 + y_2 * ncols];

				// compute weights
				T w_x1 = x_2 - (x[i] - origin_offset);
				T w_x2 = (x[i] - origin_offset) - x_1;
				T w_y1 = y_2 - (y[i] - origin_offset);
				T w_y2 = (y[i] - origin_offset) - y_1;

				T a, b;
				a = f_11 * w_x1 + f_21 * w_x2;
				b = f_12 * w_x1 + f_22 * w_x2;
				result[i] = a * w_y1 + b * w_y2;
			}
		}
	}

    template <typename T>
    void interp2_F_cx(const T* const data_r, const T* const data_i,
                      const size_t& nrows, const size_t& ncols, 
                      const T* const x, const T* const y,
                      const size_t& N, T* result_r, T* result_i,
                      const long long& origin_offset=0){

        for (auto i = 0; i < N; ++i) {

           // get coordinates of bounding grid locations
           long long x_1 = ( long long) std::floor(x[i]) - origin_offset;
           long long x_2 = x_1 + 1;
           long long y_1 = ( long long) std::floor(y[i]) - origin_offset;
           long long y_2 = y_1 + 1;
          
           // handle special case where x/y is the last element
           if ( (x[i] - origin_offset) == (nrows-1) )   { x_2 -= 1; x_1 -= 1;}
           if ( (y[i] - origin_offset) == (ncols-1) )   { y_2 -= 1; y_1 -= 1;}

           // return 0 for target values that are out of bounds
           if (x_1 < 0 | x_2 > (nrows - 1) |  y_1 < 0 | y_2 > (ncols -1)){
                result_r[i] = 0;
                result_i[i] = 0;
            } 
            else {
            
                // get the array values
                const T& f_11_r = data_r[x_1 + y_1*nrows];
                const T& f_12_r = data_r[x_1 + y_2*nrows];
                const T& f_21_r = data_r[x_2 + y_1*nrows];
                const T& f_22_r = data_r[x_2 + y_2*nrows];

                // compute weights
                T w_x1 = x_2 - (x[i] - origin_offset);
                T w_x2 = (x[i] - origin_offset) - x_1;
                T w_y1 = y_2 - (y[i] - origin_offset);
                T w_y2 = (y[i] - origin_offset) - y_1;

                T a,b;
                a = f_11_r * w_x1 + f_21_r * w_x2;
                b = f_12_r * w_x1 + f_22_r * w_x2;
                result_r[i] = a * w_y1 + b * w_y2;

                const T& f_11_i = data_i[x_1 + y_1*nrows];
                const T& f_12_i = data_i[x_1 + y_2*nrows];
                const T& f_21_i = data_i[x_2 + y_1*nrows];
                const T& f_22_i = data_i[x_2 + y_2*nrows];

                a = f_11_i * w_x1 + f_21_i * w_x2;
                b = f_12_i * w_x1 + f_22_i * w_x2;
                result_i[i] = a * w_y1 + b * w_y2;
            }
        }
    }

	template <typename T>
	void interp3_C(const T* data,
		const size_t& nrows, const size_t& ncols, const size_t& nlayers,
		const T* x, const T* y, const T* z,
		const size_t& N, T* result,
		const long long& origin_offset = 0) {

		// Assumes C style ordering for data
		for (auto i = 0; i < N; ++i) {

			// get coordinates of bounding grid locations
			long long x_1 = (long long)std::floor(x[i]) - origin_offset;
			long long x_2 = x_1 + 1;
			long long y_1 = (long long)std::floor(y[i]) - origin_offset;
			long long y_2 = y_1 + 1;
			long long z_1 = (long long)std::floor(z[i]) - origin_offset;
			long long z_2 = z_1 + 1;

			// handle special case where x, y, or z is the last element
			if ((x[i] - origin_offset) == (ncols - 1)) { x_2 -= 1; x_1 -= 1; }
			if ((y[i] - origin_offset) == (nrows - 1)) { y_2 -= 1; y_1 -= 1; }
			if ((z[i] - origin_offset) == (nlayers - 1)) { z_2 -= 1; z_1 -= 1; }

			// return 0 for target values that are out of bounds
			if (x_1 < 0 || x_2 > (ncols - 1) || y_1 < 0 || y_2 > (nrows - 1) || z_1 < 0 || z_2 > (nlayers - 1)) {
				result[i] = 0;
			}
			else {

				// precompute some stride-related constants that are reused
				const size_t layer_size = ncols * nrows;
				auto z_stride = z_1 * layer_size;
				auto y_1_stride = y_1 * ncols;
				auto y_2_stride = y_2 * ncols;

				// get the array values for the lower z slice
				const T& f_11_1 = data[z_stride + y_1_stride + x_1];
				const T& f_12_1 = data[z_stride + y_2_stride + x_1];
				const T& f_21_1 = data[z_stride + y_1_stride + x_2];
				const T& f_22_1 = data[z_stride + y_2_stride + x_2];

				// compute weights
				T w_x1 = x_2 - (x[i] - origin_offset);
				T w_x2 = (x[i] - origin_offset) - x_1;
				T w_y1 = y_2 - (y[i] - origin_offset);
				T w_y2 = (y[i] - origin_offset) - y_1;

				T a_1, b_1;
				a_1 = f_11_1 * w_x1 + f_21_1 * w_x2;
				b_1 = f_12_1 * w_x1 + f_22_1 * w_x2;

				const T F_1 = a_1 * w_y1 + b_1 * w_y2;

				// update some stride-related constants that are reused
				z_stride = z_2 * layer_size;

				// get the array values for the upper z slice
				const T& f_11_2 = data[z_stride + y_1_stride + x_1];
				const T& f_12_2 = data[z_stride + y_2_stride + x_1];
				const T& f_21_2 = data[z_stride + y_1_stride + x_2];
				const T& f_22_2 = data[z_stride + y_2_stride + x_2];

				T a_2, b_2;
				a_2 = f_11_2 * w_x1 + f_21_2 * w_x2;
				b_2 = f_12_2 * w_x1 + f_22_2 * w_x2;

				const T F_2 = a_2 * w_y1 + b_2 * w_y2;

				T w_z1 = z_2 - (z[i] - origin_offset);
				T w_z2 = (z[i] - origin_offset) - z_1;

				result[i] = F_1 * w_z1 + F_2 * w_z2;
			}
		}
	}

    template <typename T>
    void interp3_F(const T* data, 
                   const size_t& nrows, const size_t& ncols, const size_t& nlayers,
                   const T* x, const T* y, const T* z, 
                   const size_t& N, T* result,
                   const long long& origin_offset=0){
        
        // Assumes Fortran style ordering for data
        for (auto i = 0; i < N; ++i) {

            // get coordinates of bounding grid locations
            long long x_1 = ( long long) std::floor(x[i]) - origin_offset;
            long long x_2 = x_1 + 1;
            long long y_1 = ( long long) std::floor(y[i]) - origin_offset;
            long long y_2 = y_1 + 1;
            long long z_1 = ( long long) std::floor(z[i]) - origin_offset;
            long long z_2 = z_1 + 1;

            // handle special case where x,x, or z is the last element
            if ( (x[i] - origin_offset) == (nrows-1) )   { x_2 -= 1; x_1 -= 1;}
            if ( (y[i] - origin_offset) == (ncols-1) )   { y_2 -= 1; y_1 -= 1;}
            if ( (z[i] - origin_offset) == (nlayers-1) ) { z_2 -= 1; z_1 -= 1;}

            // return 0 for target values that are out of bounds
            if (x_1 < 0 | x_2 > (nrows - 1) |  y_1 < 0 | y_2 > (ncols - 1) | z_1 < 0 | z_2 > (nlayers - 1)){
                result[i] = 0;

            } 
            else {
                
                // precompute some stride-related constants that are reused
                const size_t layer_size = ncols * nrows;
                auto z_stride = z_1*layer_size;
                auto y_1_stride = y_1*nrows;
                auto y_2_stride = y_2*nrows;
                
                // get the array values for the lower z slice
                const T& f_11_1 = data[z_stride + y_1_stride + x_1];
                const T& f_12_1 = data[z_stride + y_2_stride + x_1];
                const T& f_21_1 = data[z_stride + y_1_stride + x_2];
                const T& f_22_1 = data[z_stride + y_2_stride + x_2];

                // compute weights
                T w_x1 = x_2  - (x[i] - origin_offset);
                T w_x2 = (x[i] - origin_offset) - x_1;
                T w_y1 = y_2  - (y[i] - origin_offset);
                T w_y2 = (y[i] - origin_offset) - y_1;

                T a_1, b_1;
                a_1 = f_11_1 * w_x1 + f_21_1 * w_x2;
                b_1 = f_12_1 * w_x1 + f_22_1 * w_x2;

                const T F_1 = a_1 * w_y1 + b_1 * w_y2;

                // update some stride-related constants that are reused
                z_stride = z_2*layer_size;
                
                // get the array values for the upper z slice
                const T& f_11_2 = data[z_stride + y_1_stride + x_1];
                const T& f_12_2 = data[z_stride + y_2_stride + x_1];
                const T& f_21_2 = data[z_stride + y_1_stride + x_2];
                const T& f_22_2 = data[z_stride + y_2_stride + x_2];

                T a_2, b_2;
                a_2 = f_11_2 * w_x1 + f_21_2 * w_x2;
                b_2 = f_12_2 * w_x1 + f_22_2 * w_x2;

                const T F_2 = a_2 * w_y1 + b_2 * w_y2;

                T w_z1 = z_2 - (z[i] - origin_offset);
                T w_z2 = (z[i] - origin_offset) - z_1;
                
               result[i] = F_1 * w_z1 + F_2*w_z2;
            }
        }
    }

        
    template <typename T>
    void interp3_F_cx(const T* const data_r,  const T* const data_i, 
                      const size_t& nrows, const size_t& ncols, const size_t& nlayers, 
                      const T* const x, const T* const y, const T* const z, const size_t& N, 
                      T* result_r, T* result_i,
                      const long long& origin_offset=0){
        
        // Fortran style ordering for trilinear interpolation of complex data
        for (auto i = 0; i < N; ++i) {

            // get coordinates of bounding grid locations
            long long x_1 = ( long long) std::floor(x[i])-origin_offset;
            long long x_2 = x_1 + 1;
            long long y_1 = ( long long) std::floor(y[i])-origin_offset;
            long long y_2 = y_1 + 1;
            long long z_1 = ( long long) std::floor(z[i])-origin_offset;
            long long z_2 = z_1 + 1;
            
            // handle special case where x, y, or z is the last element
            if ( (x[i] - origin_offset) == (nrows-1) )   { x_2 -= 1; x_1 -= 1;}
            if ( (y[i] - origin_offset) == (ncols-1) )   { y_2 -= 1; y_1 -= 1;}
            if ( (z[i] - origin_offset) == (nlayers-1) ) { z_2 -= 1; z_1 -= 1;}

            // return 0 for target values that are out of bounds
            if (x_1 < 0 | x_2 > (nrows - 1) |  y_1 < 0 | y_2 > (ncols - 1) | z_1 < 0 | z_2 > (nlayers - 1)){
                result_r[i] = 0;
                result_i[i] = 0;

            } 
            else {
                    const size_t layer_size = ncols * nrows;
                    auto z_stride = z_1*layer_size;
                    auto y_1_stride = y_1*nrows;
                    auto y_2_stride = y_2*nrows;
                    
                    // compute weights
                    T w_x1 = x_2  - (x[i] - origin_offset); 
                    T w_x2 = (x[i] - origin_offset) - x_1;
                    T w_y1 = y_2  - (y[i] - origin_offset);
                    T w_y2 = (y[i] - origin_offset) - y_1;

                {
                    
                    // lower Z plane, real part

                    const T& f_11_1 = data_r[z_stride + y_1_stride + x_1];
                    const T& f_12_1 = data_r[z_stride + y_2_stride + x_1];
                    const T& f_21_1 = data_r[z_stride + y_1_stride + x_2];
                    const T& f_22_1 = data_r[z_stride + y_2_stride + x_2];

                    T a_1, b_1;
                    a_1 = f_11_1 * w_x1 + f_21_1 * w_x2;
                    b_1 = f_12_1 * w_x1 + f_22_1 * w_x2;

                    const T F_1 = a_1 * w_y1 + b_1 * w_y2;

                    // upper Z plane, real part

                    z_stride = z_2*layer_size;
                    const T& f_11_2 = data_r[z_stride + y_1_stride + x_1];
                    const T& f_12_2 = data_r[z_stride + y_2_stride + x_1];
                    const T& f_21_2 = data_r[z_stride + y_1_stride + x_2];
                    const T& f_22_2 = data_r[z_stride + y_2_stride + x_2];

                    T a_2, b_2;
                    a_2 = f_11_2 * w_x1 + f_21_2 * w_x2;
                    b_2 = f_12_2 * w_x1 + f_22_2 * w_x2;

                    const T F_2 = a_2 * w_y1 + b_2 * w_y2;

                    // compute weights
                    T w_z1 = z_2 - (z[i] - origin_offset);
                    T w_z2 = (z[i] - origin_offset) - z_1;

                   result_r[i] = F_1 * w_z1 + F_2*w_z2;
                }
                
                {
                    // lower Z plane, imaginary part
                    
                    auto z_stride = z_1*layer_size;
                    const T& f_11_1 = data_i[z_stride + y_1_stride + x_1];
                    const T& f_12_1 = data_i[z_stride + y_2_stride + x_1];
                    const T& f_21_1 = data_i[z_stride + y_1_stride + x_2];
                    const T& f_22_1 = data_i[z_stride + y_2_stride + x_2];

                    T a_1, b_1;
                    a_1 = f_11_1 * w_x1 + f_21_1 * w_x2;
                    b_1 = f_12_1 * w_x1 + f_22_1 * w_x2;

                    const T F_1 = a_1 * w_y1 + b_1 * w_y2;

                    // upper Z plane, imaginary part

                    z_stride = z_2*layer_size;
                    const T& f_11_2 = data_i[z_stride + y_1_stride + x_1];
                    const T& f_12_2 = data_i[z_stride + y_2_stride + x_1];
                    const T& f_21_2 = data_i[z_stride + y_1_stride + x_2];
                    const T& f_22_2 = data_i[z_stride + y_2_stride + x_2];

                    T a_2, b_2;
                    a_2 = f_11_2 * w_x1 + f_21_2 * w_x2;
                    b_2 = f_12_2 * w_x1 + f_22_2 * w_x2;

                    const T F_2 = a_2 * w_y1 + b_2 * w_y2;

                    T w_z1 = z_2 - (z[i] - origin_offset);
                    T w_z2 = (z[i] - origin_offset) - z_1;

                   result_i[i] = F_1 * w_z1 + F_2*w_z2;
                }                        
            }
        }
    }
}
#endif //SPLINTERP_H
