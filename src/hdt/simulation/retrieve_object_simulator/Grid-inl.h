#ifndef Grid_inl_h
#define Grid_inl_h

#include "Grid.h"

template <typename T>
Grid<T>::Grid() : width_(0), height_(0), data_(nullptr), nocompact_(nullptr)
{
}

template <typename T>
void Grid<T>::resize(int width, int height)
{
    clear();
    this->data_ = new T[width * height];
    this->width_ = width;
    this->height_ = height;
};

template <typename T>
Grid<T>::~Grid()
{
    clear();
}

template <typename T>
int Grid<T>::width() const
{
    return width_;
}

template <typename T>
int Grid<T>::height() const
{
    return height_;
}

template <typename T>
const T* Grid<T>::data() const
{
    return this->data_;
}

template <typename T>
T* Grid<T>::data()
{
    return this->data_;
}

template <typename T>
const T** Grid<T>::data_nocompact() const
{
    const_cast<Grid<T>*>(this)->create_noncompact();
    return nocompact_;
}

template <typename T>
T** Grid<T>::data_nocompact()
{
    create_noncompact();
    return nocompact_;
}

template <typename T>
T& Grid<T>::operator()(int x, int y)
{
    return this->data_[coord_to_index(x, y)];
}

template <typename T>
const T& Grid<T>::operator()(int x, int y) const
{
    return this->data_[coord_to_index(x, y)];
}

template <typename T>
typename Grid<T>::iterator Grid<T>::begin()
{
    return GridIterator(this, 0, 0);
}

template <typename T>
typename Grid<T>::iterator Grid<T>::end()
{
    return GridIterator(this, 0, this->height_);
}

template <typename T>
typename Grid<T>::const_iterator Grid<T>::begin() const
{
    return GridIterator(this, 0, 0);
}

template <typename T>
typename Grid<T>::const_iterator Grid<T>::end() const
{
    return GridIterator(this, 0, this->height_);
}

template <typename T>
void Grid<T>::clear()
{
    // clear the main data array
    if (this->data_) {
        delete [] this->data_;
        this->data_ = nullptr;
    }

    // clear the noncompact data array
    if (this->nocompact_) {
        delete [] this->nocompact_;
        this->nocompact_ = nullptr;
    }

    this->width_ = 0;
    this->height_ = 0;
}

template <typename T>
inline int Grid<T>::coord_to_index(int x, int y) const
{
    return y + x * this->height_;
}

template <typename T>
void Grid<T>::create_noncompact()
{
    if (this->data_ && !this->nocompact_) {
        nocompact_ = new T*[this->width_];
        for (int x = 0; x < this->width_; ++x) {
            nocompact_[x] = &this->operator()(x, 0);
        }
    }
}

template <typename T>
Grid<T>::GridIterator::GridIterator() :
    grid_(nullptr), x_(0), y_(0)
{ }

template <typename T>
Grid<T>::GridIterator::GridIterator(Grid* grid, int start_x, int start_y) :
    grid_(grid), x_(start_x), y_(start_y)
{ }

template <typename T>
Grid<T>::GridIterator::GridIterator(const GridIterator& other) :
    grid_(other.grid_),
    x_(other.x_),
    y_(other.y_)
{ }

template <typename T>
typename Grid<T>::GridIterator::GridIterator& Grid<T>::GridIterator::operator=(const GridIterator& rhs)
{
    if (this != &rhs) {
        grid_ = rhs.grid_; x_ = rhs.x_; y_ = rhs.y_;
    }
    return *this;
}

template <typename T>
typename Grid<T>::GridIterator::GridIterator& Grid<T>::GridIterator::operator++()
{
    advance();
    return *this;
}

template <typename T>
typename Grid<T>::GridIterator::GridIterator& Grid<T>::GridIterator::operator++(int)
{
    advance();
    return *this;
}

template <typename T>
typename Grid<T>::GridIterator::GridIterator& Grid<T>::GridIterator::operator--()
{
    retreat();
    return *this;
}

template <typename T>
typename Grid<T>::GridIterator::GridIterator& Grid<T>::GridIterator::operator--(int)
{
    retreat();
    return *this;
}

template <typename T>
bool Grid<T>::GridIterator::operator==(const GridIterator& rhs)
{
    return grid_ == rhs.grid_ && x_ == rhs.x_ && y_ == rhs.y_;
}

template <typename T>
bool Grid<T>::GridIterator::operator!=(const GridIterator& rhs)
{
    return !this->operator==(rhs);
}

template <typename T>
T& Grid<T>::GridIterator::operator*()
{
    return (*grid_)(x_, y_);
}

template <typename T>
T& Grid<T>::GridIterator::operator->()
{
    return (*grid_)(x_, y_);
}

template <typename T>
void Grid<T>::GridIterator::advance()
{
    if (!this->grid_) {
        return;
    }

    if (this->grid_->width() == 0 || x_ == this->grid_->width() - 1) {
        ++y_;
        x_ = 0;
    }
    else {
        ++x_;
    }
}

template <typename T>
void Grid<T>::GridIterator::retreat()
{
    if (!this->grid_) {
        return;
    }

    if (x_ == 0) {
        if (y_ != 0) {
            --y_;
            if (grid_->width() != 0) {
                x_ = grid_->width() - 1;
            }
//          else { } // do nothing
        }
//      else { } // do nothing
    }
    else {
        --x_;
    }
}

#endif
