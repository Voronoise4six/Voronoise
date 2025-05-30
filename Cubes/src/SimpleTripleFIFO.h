#ifndef SIMPLE_TRIPLE_FIFO_H
#define SIMPLE_TRIPLE_FIFO_H

class SimpleTripleFIFO {
private:
    struct Triple {
        int a, b, c;
    };
    
    Triple* buffer;
    unsigned int size;
    unsigned int head;
    unsigned int tail;
    unsigned int count;

public:
    // Constructor/Destructor
    SimpleTripleFIFO(unsigned int buffer_size);
    ~SimpleTripleFIFO();

    // Core operations
    bool push(int a, int b, int c);
    bool pop(int &a, int &b, int &c);

    // Status methods
    bool isEmpty() const;
    bool isFull() const;
    unsigned int available() const;
};

#endif