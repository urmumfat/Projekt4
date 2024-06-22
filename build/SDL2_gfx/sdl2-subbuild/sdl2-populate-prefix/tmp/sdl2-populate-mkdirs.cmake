# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/SDL-2.24.2"
  "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-build"
  "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix"
  "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/tmp"
  "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp"
  "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src"
  "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/adamj/Desktop/tp_projekt_4/build/SDL2_gfx/sdl2-subbuild/sdl2-populate-prefix/src/sdl2-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
