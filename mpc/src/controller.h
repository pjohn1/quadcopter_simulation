#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Python.h>
#include <vector>
#include <iostream>

class Controller {
public:
    Controller();
    ~Controller();

    std::vector<double> optimizeControl(int N, const std::vector<double>& X0, const std::vector<double>& goal);

private:
    PyObject *pModule;  // Python module
    PyObject *pFunc;    // Python function
};

#endif // CONTROLLER_H
