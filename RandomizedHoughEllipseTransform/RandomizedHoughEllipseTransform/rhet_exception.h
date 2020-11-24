#pragma once
#include <iostream>
#include <stdio.h>
#include <string.h>

class HoughException : public std::exception {
public:
    virtual const char* what() const throw() override {
        return "Something goes wrong!";
    }
};

class NoEdgeException : public HoughException {
    const char* what() const throw() override {
        return "After Canny there are no Edges";
    }
};

class InputException : public HoughException {
public:
    std::string message;
    InputException(std::string e) {
        message = e;
    }

    const char* what() const throw() override {
        if (message == "phase") return "Invalid phase image!";
        if (message == "mask") return "Invalid mask image!";
        return "Input Error";
    }
};

class EmptyException : public HoughException {
public:
    std::string message;
    EmptyException(std::string e) {
        message = e;
    }

    const char* what() const throw() override {
        if (message == "phase") return "Cannot Find Phase Image!";
        if (message == "mask") return "Cannot Find Mask Image!";
        return "Empty Error";
    }
};