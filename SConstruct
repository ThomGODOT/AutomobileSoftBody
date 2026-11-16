#!/usr/bin/env python

from SCons.Script import *

libname = "mobile_softbody"
projectdir = "project"

localEnv = Environment(tools=["default"], PLATFORM="")
env = localEnv.Clone()

env = SConscript(
    "godot-cpp/SConstruct",
    {
        "api_version": "4.7",
        "env": env,
    }
)

env.Append(CPPPATH=["src/"])
env.Append(CXXFLAGS=["-std=c++17"])

sources = Glob("src/*.cpp")

lib_filename = "{}{}{}{}".format(
    env.subst("$SHLIBPREFIX"),
    libname,
    env["suffix"],
    env.subst("$SHLIBSUFFIX")
)

library = env.SharedLibrary(
    "{}/addons/mobile_softbody/bin/{}".format(
        projectdir,
        lib_filename
    ),
    source=sources,
)

Default(library)
