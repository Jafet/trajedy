# The Trajedy programming language

This repository contains a specification document, interpreter, and example programs.

## Specification

The [esowiki](https://esolangs.org) has a [version of the specification](https://esolangs.org/wiki/Trajedy), which may be outdated or indated (i.e. more advanced) relative to the specification here.

## Reference interpreter

To compile the interpreter (`trajedy`), you need a relatively modern C++ compiler and the [GNU MP library](http://gmplib.org).

The interpreter uses a small library for command-line parsing, which should be fetched using `git submodule`.

In addition to those, the graphical interpreter (`trajedebug`) requires [gtkmm](http://gtkmm.org) and [cairomm](http://cairographics.org/cairomm). This interpreter has basic debugging functions, and is essential if you want to see how the examples work.

The included Makefile should work for GCC on a Debian-like system. For Debian, the library packages are:

```
libgmp-dev libgtkmm-3.0-dev libcairomm-1.0-dev
```

## Example programs

* [`loop`](examples/loop.tra): trivial (and rather degenerate) infinite loop.
* [`loop2`](examples/loop2.tra): less trivial triangle-shaped loop.
* [`loop3`](examples/loop3.tra): interesting loop that goes through an infinite number of states.
* [`spiral-in`](examples/spiral-in.tra), [`spiral-out`](examples/spiral-out.tra): more loops. These have asymptotic spiralling shapes.
* [`hello`](examples/hello.tra): simple hello world program.
* [`hello2`](examples/hello2.tra): compact hello world program, split over multiple lines.
* [`truth`](examples/truth.tra): [truth-machine](https://esolangs.org/wiki/Truth-machine), demonstrating basic I/O and control flow.
* [`cat`](examples/cat.tra): copies its input to output. The end-of-input detection makes this program rather large and complicated.
* [`reverse01`](examples/reverse01.tra): reads a string of `0` and `1` characters, and outputs its reverse. Essentially stores a stack in the pointer state.
* [`reverse01-with-skip`](examples/reverse01-with-skip.tra): an earlier version of `reverse01`, that relied on a less inelegant construction (skipping squares with `?`).
* [`div`](examples/div.tra): a test bed for divisibility testing using a _zig-zag device_. The gory details are part of the Turing-completeness proof. Aside from that, you can simply enjoy various zig-zag patterns by moving the second `X` around.