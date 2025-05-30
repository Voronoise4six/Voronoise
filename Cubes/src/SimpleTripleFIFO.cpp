#include "SimpleTripleFIFO.h"

// Constructor
SimpleTripleFIFO::SimpleTripleFIFO(unsigned int buffer_size) {
    size = buffer_size;
    buffer = new Triple[size];
    head = 0;
    tail = 0;
    count = 0;
}

// Destructor
SimpleTripleFIFO::~SimpleTripleFIFO() {
    delete[] buffer;
}

// Add three integers to the buffer
bool SimpleTripleFIFO::push(int a, int b, int c) {
    if (count == size) {
        return false; // Buffer full
    }
    
    buffer[tail].a = a;
    buffer[tail].b = b;
    buffer[tail].c = c;
    
    tail = (tail + 1) % size;
    count++;
    
    return true;
}

// Retrieve three integers from the buffer
bool SimpleTripleFIFO::pop(int &a, int &b, int &c) {
    if (count == 0) {
        return false; // Buffer empty
    }
    
    a = buffer[head].a;
    b = buffer[head].b;
    c = buffer[head].c;
    
    head = (head + 1) % size;
    count--;
    
    return true;
}

// Check if buffer is empty
bool SimpleTripleFIFO::isEmpty() const {
    return count == 0;
}

// Check if buffer is full
bool SimpleTripleFIFO::isFull() const {
    return count == size;
}

// Get number of triple entries currently in buffer
unsigned int SimpleTripleFIFO::available() const {
    return count;
}