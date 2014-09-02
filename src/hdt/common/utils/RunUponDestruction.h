#ifndef RunUponDestruction_h
#define RunUponDestruction_h

#include <functional>

class RunUponDestruction
{
public:

    RunUponDestruction(const std::function<void()>& fun) : fun_(fun) { }
    ~RunUponDestruction() { fun_(); }

private:

    std::function<void()> fun_;
};

#endif
