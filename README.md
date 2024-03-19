# detect-ships
Script to run ship detection on images with ships on the ocean. Assumptions have been made to ease simplicity of solution. To read full explanation, read the main.hpp for function design choices

## Pre-reqs
Requires libopencv-dev, g++ and cmake. Run setup.sh if these aren't installed

## How to run
To run the compiled code, simply type the following command
`./Detect-Ships`

If you want to make changes to the code and run, run these commands
```
cmake .
cmake --build .
./Detect-Ships
```
