# UIC Campus Map Navigation

This program is designed to navigate the University of Illinois Chicago (UIC) campus map using OpenStreetMap data. It allows users to find the shortest distance between two buildings and calculate the destination building equidistant from them using Dijkstra's shortest-path algorithm. The implementation utilizes a graph structure stored as adjacency lists.

## Getting Started

To use this program, follow these steps:

1. Clone or download the repository containing the source code.

2. Compile the C++ program using a C++ compiler such as g++:

    ```
    g++ application.cpp -o application
    ```

3. Run the compiled executable:

    ```
    ./application
    ```

4. Follow the prompts to input the buildings for Person 1 and Person 2.

## Features

- **Building Search:** The program allows users to search for buildings by partial or full name, as well as abbreviations.

- **Shortest Path Calculation:** It calculates the shortest distance between two buildings using Dijkstra's shortest-path algorithm.

- **Destination Building Calculation:** The program determines a destination building equidistant from two specified buildings.

## References

- TinyXML: Used for parsing XML files.
- OpenStreetMpa: Provides map data 

## Author

- Javid Uddin

## Acknowledgements

- Adam T Koehler, PhD
- Joe Hummel, PhD
- University of Illinois at Chicago

