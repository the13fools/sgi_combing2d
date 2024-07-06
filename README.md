Use this starter project to accelerate your geometry processing research, yesterday. 

# 🤝 libigl 🤝 TinyAD 🤝 Polyscope 🤝 Physics thread 🤝 foliations 🤝

Welcome to the repository for the wonderful, wonderous, GPGPT!

# The General Purpose Geometry Processing Toolkit!  

This is an example project which minimally mixes a maximal number of independently useful libraries.  Put ur rings together captin planet.  

You might be confusing us with (gptoolbox)[https://github.com/alecjacobson/gptoolbox], we don't support that yet though.  

In this repo, we mix [libigl](https://github.com/libigl/libigl/) and
[TinyAD](https://github.com/patr-schm/TinyAD) and [polyscope](polyscope.run) and [a physics thread](https://github.com/evouga/libigl-example-physics-project).

On top of this, we also include our own heavily gpt generated example library for performing a geometric optimization problem and logging it's state to file for later visualization.  

If you want higher fidelity, by a path tracer like the one in blender, using scripts like this (toolbox)[https://github.com/HTDerekLiu/BlenderToolbox].

animations can then be produced using some ffmpeg scripts we also bake in.  


## Compile

Compile this project using the standard cmake routine:

    mkdir build
    cd build
    cmake ..
    make

This should find and build the dependencies and create a `bin/##` binary.

## Run

From within the `build` directory just issue:

    ./example

A glfw app should launch displaying an animating Armadillo parametrization.

![](armadillo.gif)

_Derived from
[parametrization_libigl.cc](https://github.com/patr-schm/TinyAD-Examples/blob/main/apps/parametrization_libigl.cc)_


## Ordering

TinyAD operates most conveniently on nodal vector values (e.g., vertex positions in `V`). If `V` contains nodal vectors **per row**, then regardless of whether `V` is stored as column-major (e.g., `Eigen::MatrixXd`) or row-major (e.g., `Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>`) the internal order of tiny-ad will correspond to [non-standard](https://en.wikipedia.org/wiki/Vectorization_(mathematics)) **row-major vectorization**.

That is, if
```
V = [
  x₀ y₀ z₀
  x₁ y₁ z₁
  …
  xₙ yₙ zₙ
  ]
```

Then TinyAD will vectorize this into
```
x = [
  x₀
  y₀
  z₀
  x₁
  y₁
  z₁
  …
  xₙ
  yₙ
  zₙ
  ]
```

And use a corresponding ordering for gradients and Hessians.

This is ignorable if you're using the provided `func.x_from_data` and `func.x_to_data`. However, if you're mixing in your own gradients, Hessians, constraint projections, subspace bases, then take care!


# MAC BUILD DETAILS 

fresh install details: 
brew install cmake eigen suitesparse llvm libomp

------------

Oh boy, this was quite tricky to pin down.  Maybe not needed here though.  

Thank you eerii .  Anonomous internet comments really came through here.  

https://github.com/glfw/glfw/issues/1743#issuecomment-1229177189

For now until we improve the cmake file we added the following to our ~/.zshrc

```
export CC=$(which clang)
export CXX=$(which g++-13)
export OpenMP_ROOT=$(brew --prefix)/opt/libomp
```

BTW, todo: how to enable gdb on mac https://stackoverflow.com/a/10441587



