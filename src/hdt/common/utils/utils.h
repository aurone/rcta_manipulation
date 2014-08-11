#ifndef utils_h
#define utils_h

template <typename T>
const T& clamp(const T& t, const T& min, const T& max)
{
    return t > max ? max : t < min ? min : t;
}

#endif

