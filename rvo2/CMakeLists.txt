#
# CMakeLists.txt
# RVO2 Library
#
# Copyright (c) 2008-2010 University of North Carolina at Chapel Hill.
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software and its
# documentation for educational, research, and non-profit purposes, without fee,
# and without a written agreement is hereby granted, provided that the above
# copyright notice, this paragraph, and the following four paragraphs appear in
# all copies.
#
# Permission to incorporate this software into commercial products may be
# obtained by contacting the authors <geom@cs.unc.edu> or the Office of
# Technology Development at the University of North Carolina at Chapel Hill
# <otd@unc.edu>.
#
# This software program and documentation are copyrighted by the University of
# North Carolina at Chapel Hill. The software program and documentation are
# supplied "as is," without any accompanying services from the University of
# North Carolina at Chapel Hill or the authors. The University of North Carolina
# at Chapel Hill and the authors do not warrant that the operation of the
# program will be uninterrupted or error-free. The end-user understands that the
# program was developed for research purposes and is advised not to rely
# exclusively on the program for any reason.
#
# IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
# AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
# SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
# CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
#
# THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
# DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY STATUTORY
# WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS"
# BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS
# HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
# MODIFICATIONS.
#
# Please send all bug reports to <geom@cs.unc.edu>.
#
# The authors may be contacted via:
#
# Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
# Dept. of Computer Science
# 201 S. Columbia St.
# Frederick P. Brooks, Jr. Computer Science Bldg.
# Chapel Hill, N.C. 27599-3175
# United States of America
#
# <http://gamma.cs.unc.edu/RVO2/>
#

cmake_minimum_required(VERSION 2.8.3)
project(rvo2)

find_package(catkin)
find_package(OpenMP REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES rvo2)
 include_directories(include/RVO)

add_library(rvo2 
  src/Agent.cpp src/KdTree.cpp src/Obstacle.cpp src/RVOSimulator.cpp)

target_link_libraries(rvo2 OpenMP::OpenMP_CXX)
