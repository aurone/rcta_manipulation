#ifndef Grid_h
#define Grid_h

template <typename T>
class Grid
{
public:

    struct GridIterator
    {
        GridIterator();
        GridIterator(Grid* grid_, int start_x, int start_y);
        GridIterator(const GridIterator& other);
        GridIterator& operator=(const GridIterator& rhs);

        GridIterator& operator++();
        GridIterator& operator++(int);
        GridIterator& operator--();
        GridIterator& operator--(int);
        bool operator==(const GridIterator& rhs);
        bool operator!=(const GridIterator& rhs);
        T& operator*();
        T& operator->();

    private:

        Grid* grid_;
        int x_, y_;

        void advance();
        void retreat();
    };

    typedef T value_type;
    typedef GridIterator iterator;
    typedef const GridIterator const_iterator;

    Grid();
    ~Grid();

    void resize(int width, int height);

    int width() const;
    int height() const;

    const T* data() const;
    T* data();
    const T** data_nocompact() const;
    T** data_nocompact();

    T& operator()(int x, int y);
    const T& operator()(int x, int y) const;

    iterator begin();
    iterator end();

    const_iterator begin() const;
    const_iterator end() const;

    void clear();

private:

    int width_;
    int height_;
    T* data_;
    T** nocompact_;

    inline int coord_to_index(int x, int y) const;
    void create_noncompact();
};

#include "Grid-inl.h"

#endif
