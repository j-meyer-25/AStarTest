# AStarTest

![Rust Version](https://img.shields.io/badge/Rust-1.77.2%2B-brown)

#### Josh Meyer, Alex Burkholder

A* Pathfinding Algorithm

> https://en.wikipedia.org/wiki/A*_search_algorithm

This is a simple Rust implementation of A*, which searches through a graph data structure to construct a path between a start and an end point.
A* uses a heuristic function to guide the algorithm towards the end point in an efficient manner.

## Specifics

This implementation uses Euclidean (straight-line) distance for the heuristic and is based on a fixed-size binary array to represent the graph, where 1's are pathable spaces.
This is intended for a VS Code environment.

## Notes

This is a test program written for a school project, and to strengthen Rust skills. It may not be viable for new work due to the fixed-size limitation. It has not been rigorously tested.
