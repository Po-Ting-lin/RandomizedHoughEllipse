//
// Created by ptl on 2020-03-10.
//

#ifndef RANDOMIZEDHOUGHELLIPSE_HOUGHEXCEPTION_H
#define RANDOMIZEDHOUGHELLIPSE_HOUGHEXCEPTION_H

#include <iostream>

class HoughException: public std::exception{
public:
    virtual const char* what() const throw() override {
        return "Something goes wrong!";
    }
};

class NoEdgeExcception: public HoughException{
    const char* what() const throw() override {
        return "After Canny there are no Edges";
    }
};


#endif //RANDOMIZEDHOUGHELLIPSE_HOUGHEXCEPTION_H
