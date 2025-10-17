[![Build Status](https://travis-ci.org/david-m-rosen/SE-Sync.svg?branch=master)](https://travis-ci.org/david-m-rosen/SE-Sync)

# SE-Sync


**SE-Sync** is a *certifiably correct* algorithm for performing *synchronization over the special Euclidean group*: estimate the values of a set of unknown *poses* (positions and orientations in Euclidean space) given noisy measurements of a subset of their pairwise relative transforms.  This problem frequently arises in the context of 2D and 3D geometric estimation; for example, the foundational problems of [pose-graph SLAM](http://domino.informatik.uni-freiburg.de/teaching/ws11/robotics2/pdfs/ls-slam-tutorial.pdf) (in robotics), [camera motion estimation](http://cmp.felk.cvut.cz/ftp/articles/pajdla/Martinec-Pajdla-CVPR-2007.pdf) (in computer vision), and [sensor network localization](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3741000/) (in distributed sensing) all require synchronization over the special Euclidean group.  SE-Sync improves upon prior methods by exploiting a novel (convex) *semidefinite relaxation* of the special Euclidean synchronization problem to directly search for *globally optimal* solutions, and is capable of producing a *computational certificate of correctness* (global optimality) in the (typical) case that a global minimizer is found.

A detailed description of the algorithm and its implementation can be found in our [journal article](https://github.com/david-m-rosen/SE-Sync/blob/master/references/SE-Sync%20-%20A%20certifiably%20correct%20algorithm%20for%20synchronization%20over%20the%20special%20Euclidean%20group.pdf) and its companion [technical report](https://github.com/david-m-rosen/SE-Sync/blob/master/references/SE-Sync%20-%20A%20Certifiably%20Correct%20Algorithm%20for%20Synchronization%20over%20the%20Special%20Euclidean%20Group.pdf).



## Getting Started

### MATLAB

To use the MATLAB implementation of SE-Sync, simply place the 'MATLAB' folder in any convenient (permanent) location, and then run the script MATLAB/import_SE_Sync.m.  Congrats!  SE-Sync is now ready to go :-).  For a minimal working example, see [MATLAB/examples/main.m](https://github.com/david-m-rosen/SE-Sync/blob/master/MATLAB/examples/main.m)

### C++

The C++ implementation of SE-Sync can be built and exported as a CMake project.  For a minimal working example, see [C++/examples/main](https://github.com/david-m-rosen/SE-Sync/blob/master/C%2B%2B/examples/main.cpp), which provides a simple command-line utility for processing .g2o files.

#### C++ quick installation guide

### Linux
The following installation instructions have been verified on Ubuntu 22.04:

*Step 1:*  Install dependencies
```
$ sudo apt-get install build-essential cmake-gui libeigen3-dev liblapack-dev libblas-dev libsuitesparse-dev
```

*Step 2:*  Clone the repository
```
$ git clone https://github.com/david-m-rosen/SE-Sync.git SESync
```

*Step 3:*  Initialize Git submodules
```
$ cd SESync
$ git submodule init
$ git submodule update
```

*Step 4:*  Create build directory
```
$ cd C++ && mkdir build
```

*Step 5:*  Configure build and generate Makefiles
```
$ cd build && cmake ..
```

*Step 6:*  Build code
```
$ make -j
```

*Step 7:*  Run the example command-line utility on some tasty data :-D!
```
$ cd bin
$ ./SE-Sync ../../../data/sphere2500.g2o 
```

### Mac
The following installation instructions have been verified on MacOS 14.3.1

Step 1: Install dependencies
```
brew install cmake eigen gmp metis gcc
```

Step 2: Create a new folder 
```
mkdir SE-Sync && cd SE-Sync
```

Step 3: Install older version SuiteSparse package
```
curl -LO https://github.com/DrTimothyAldenDavis/SuiteSparse/archive/refs/tags/v5.10.1.tar.gz

tar -xzf v5.10.1.tar.gz

cd SuiteSparse-5.10.1
```

Step 4: Compile SuiteSparse
```
CMAKE_OPTIONS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5" make library \ BLAS="-framework Accelerate" \ LAPACK="-framework Accelerate" \ MY_METIS_LIB="-L$(brew --prefix)/lib -lmetis" \ MY_METIS_INC="$(brew --prefix)/include" \ CFLAGS="-I$(brew --prefix)/include" \ LDFLAGS="-L$(brew --prefix)/lib -L$PWD/lib" \ INSTALL=$PWD
```

```
make install \ INSTALL=$PWD \ BLAS="-framework Accelerate" \ LAPACK="-framework Accelerate" \ MY_METIS_LIB="-L$(brew --prefix)/lib -lmetis" \ MY_METIS_INC="$(brew --prefix)/include" \ CFLAGS="-I$(brew --prefix)/include" \ LDFLAGS="-L$(brew --prefix)/lib"
```

You may get an error that looks like this

```
CMake Error at cmake_install.cmake:41 (file):
  file INSTALL cannot copy file
  "/Users/natashanicholas/Desktop/shanefolder/SuiteSparse-5.10.1/GraphBLAS/build/libgraphblas.5.0.5.dylib"
  to "/usr/local/lib/libgraphblas.5.0.5.dylib": Permission denied.
make[2]: *** [install] Error 1
make[1]: *** [install] Error 2
make: *** [install] Error 2
```
You can ignore this, to ensure that the make command was successful run the following command
```
ls include | grep -i spqr
```
You should see something like this 
```
spqr.hpp
```

Also run this command 
```
ls lib | grep -i spqr
```
You should see something like this
```
libspqr.2.0.9.dylib
libspqr.2.dylib
libspqr.dylib
```

Step 5: Clone SE-Sync repo
```
cd ..
git clone https://github.com/david-m-rosen/SE-Sync.git
```

Step 6: Initialize Git submodules
```
cd SE-Sync
git submodule init
git submodule update
```

Step 7: Create build directory
```
cd C++ && mkdir build
cd build
```

Step 8: Configure build and generate Makefiles
```
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_C_COMPILER=$(brew --prefix gcc)/bin/gcc-15 -DCMAKE_CXX_COMPILER=$(brew --prefix gcc)/bin/g++-15 -DCMAKE_PREFIX_PATH="$(brew --prefix eigen);$PWD/../../../SuiteSparse-5.10.1;$(brew --prefix llvm);$(brew --prefix openblas)" -DCMAKE_CXX_FLAGS="-I$PWD/../../../SuiteSparse-5.10.1/include" -DCMAKE_BUILD_TYPE=Release
```

Step 9: Build code
```
make -j$(sysctl -n hw.logicalcpu)
```

*Step 10:*  Run the example command-line utility on some tasty data :-D!
```
$ cd bin
$ ./SE-Sync ../../../data/sphere2500.g2o 
```




### Python

Python bindings for the C++ SE-Sync library can also be built using [pybind11](https://pybind11.readthedocs.io/en/stable/index.html).  To do so, install the additional Python dependencies using the command:

### Linux

```
$ sudo apt-get install python3 python3-numpy python3-matplotlib python3-dev pybind11-dev jupyter-notebook 
```

### Mac

```
brew install pybind11
pip3 install numpy matplotlib jupyter notebook
```



and then set `BUILD_PYTHON_BINDINGS` when configuring the CMake project.  See this [notebook](https://github.com/david-m-rosen/SE-Sync/blob/master/C%2B%2B/examples/PySESync.ipynb) for a minimal working example demonstrating the use of SE-Sync's Python interface.

## References

We are making this software freely available in the hope that it will be useful to others. If you use SE-Sync in your own work, please [cite](https://github.com/david-m-rosen/SE-Sync/blob/master/references/SE-Sync%20-%20A%20certifiably%20correct%20algorithm%20for%20synchronization%20over%20the%20special%20Euclidean%20group.pdf) [our](https://github.com/david-m-rosen/SE-Sync/blob/master/references/A%20Certifiably%20Correct%20Algorithm%20for%20Synchronization%20over%20the%20Special%20Euclidean%20Group.pdf) [papers](https://github.com/david-m-rosen/SE-Sync/blob/master/references/Accelerating%20Certifiable%20Estimation%20with%20Preconditioned%20Eigensolvers.pdf):

```
@article{Rosen2019SESync,
title = {{SE-Sync}:  A Certifiably Correct Algorithm for Synchronization over the Special {Euclidean} Group},
author = {Rosen, D.M. and Carlone, L. and Bandeira, A.S. and Leonard, J.J.},
journal = {Intl. J. of Robotics Research},
volume = {38},
number = {2--3},
pages = {95--125},
month = mar,
year = {2019},
}

@techreport{Rosen2017SESync,
title = {{SE-Sync}: A Certifiably Correct Algorithm for Synchronization over the Special {Euclidean} Group},
author = {Rosen, D.M. and Carlone, L. and Bandeira, A.S. and Leonard, J.J.},
institution = {Computer Science and Artificial Intelligence Laboratory, Massachusetts Institute of Technology},
address = {Cambridge, MA},
number = {MIT-CSAIL-TR-2017-002},
year = {2017},
month = feb,
}

@inproceedings{Rosen2016Certifiably,
title = {A Certifiably Correct Algorithm for Synchronization over the Special {Euclidean} Group},
author = {Rosen, D.M. and Carlone, L. and Bandeira, A.S. and Leonard, J.J.},
booktitle = {Intl. Workshop on the Algorithmic Foundations of Robotics (WAFR)},
month = dec,
year = {2016},
address = {San Francisco, CA},
}

@unpublished{Rosen2017Computational,
title = {Computational Enhancements for Certifiably Correct {SLAM}},
author = {Rosen, D.M. and Carlone, L.},
note = {Presented at the International Conference on Intelligent Robots and Systems (IROS) in the workshop ``Introspective Methods for Reliable Autonomy"},
month = sep,
year = {2017},
}

@misc{Rosen2022Accelerating,
  title = {Accelerating Certifiable Estimation with Preconditioned Eigensolvers},
  author = {Rosen, David M.},
  month = may,
  year = {2022},
  publisher = {arXiv},
  doi = {10.48550/ARXIV.2207.05257},
  url = {https://arxiv.org/abs/2207.05257},
}
```

and the following [paper](https://link.springer.com/article/10.1007/s10208-005-0179-9) of Absil et al., which describes the Riemannian trust-region (RTR) method that SE-Sync employs:

```
@article{Absil2007Trust,
title = {Trust-Region Methods on {Riemannian} Manifolds},
author = {Absil, P.-A. and Baker, C.G. and Gallivan, K.A.},
journal = {Found.\ Comput.\ Math.},
volume = {7},
number = {3},
pages = {303--330},
year = {2007},
month = jul,
}
```

If you use the MATLAB implementation of SE-Sync, please also cite the following [reference](http://www.jmlr.org/papers/volume15/boumal14a/boumal14a.pdf) for the [Manopt toolbox](https://www.manopt.org/), which provides the MATLAB implementation of RTR that the SE-Sync toolbox employs:

```
@article{Boumal2014Manopt,
  title={{Manopt}, a {MATLAB} Toolbox for Optimization on Manifolds.},
  author={Boumal, N. and Mishra, B. and Absil, P.-A. and Sepulchre, R.},
  journal={Journal of Machine Learning Research},
  volume={15},
  number={1},
  pages={1455--1459},
  year={2014}
}
```


## Copyright and License 

The C++ and MATLAB implementations of SE-Sync contained herein are copyright (C) 2016-2022 by David M. Rosen, and are distributed under the terms of the GNU Lesser General Public License (LGPL) version 3 (or later).  Please see the [LICENSE](https://github.com/david-m-rosen/SE-Sync/blob/master/LICENSE) for more information.

Contact: drosen2000@gmail.com
