# ROS Code Style Guide Summary

## 1. Code Style Guide

Open-source code is a collective effort by a community, and consistent coding standards are essential for effective collaboration. By adhering to a standardized code style, developers can reduce errors, simplify code reviews, and enhance overall code readability and maintainability. Although initially cumbersome, following a code style guide minimizes subjective decisions during coding and helps avoid potential errors and issues specific to programming languages.

For ROS (Robot Operating System), which is also a collaborative effort, the community has established style guides and rules, such as the `ROS 2 Developer Guide` and `ROS Enhancement Proposals (REPs)`. These guidelines ensure a consistent code style across various programming languages used in ROS. The community follows popular and well-established styles, not arbitrary or independent ones, and provides tools to help developers ensure compliance with these guidelines.

## 2. Basic Naming Conventions

The document outlines three main naming conventions:

- **CamelCased**: Used for types, classes, structs, and enums.
- **snake_case**: Used for file names (all lowercase), packages, interfaces, namespaces, variables, functions, and methods.
- **ALL_CAPITALS**: Reserved for constants and macros.

Certain files have specific names that do not follow these conventions, such as `package.xml`, `CMakeLists.txt`, `README.md`, etc.

## 3. C++ Style

The C++ code style in ROS 2 is based on the widely-used Google C++ Style Guide, with some modifications to suit ROS. Key points include:

- Adherence to the C++14 Standard.
- Line length should not exceed 100 characters.
- Specific naming conventions are followed, with `CamelCased` for types and classes, `snake_case` for files and functions, and `ALL_CAPITALS` for constants and macros.
- Indentation uses 2 spaces (no tabs), and consistent brace usage is required for control structures.
- Comments should use `/** */` for documentation and `//` for inline comments.
- Linters like `ament_cpplint` and `ament_uncrustify` are recommended for automatic style checking, with `ament_cppcheck` for static code analysis.

## 4. Python Style

The Python code style adheres to PEP 8, a well-known style guide for Python:

- Python 3 (3.5 or higher) is the standard.
- Line length should not exceed 100 characters.
- Naming conventions include `CamelCased` for types and classes, `snake_case` for files and functions, and `ALL_CAPITALS` for constants.
- Indentation uses 4 spaces (no tabs).
- Braces and appropriate spacing follow PEP 8 guidelines.
- Docstring conventions follow PEP 257, and linters like `ament_flake8` are recommended for checking code style.

## 5. Other Languages

- **C**: Follows the C99 Standard with PEP-7 as a reference.
- **JavaScript**: Follows the Airbnb JavaScript Style Guide.

## 6. Conclusion

The best way to learn and apply code style is by reading well-written code and consistently applying the style in your work. The document recommends using specific tools to ensure compliance with C++ and Python coding standards and refers to Google C++ Style Guide and PEP-8 for more detailed information.