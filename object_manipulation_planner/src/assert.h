#ifndef OBJECT_MANIPULATION_PLANNER_ASSERT_H
#define OBJECT_MANIPULATION_PLANNER_ASSERT_H

#include <exception>

#define SMPL_ASSERT(cond, msg) do { if (!(cond)) throw std::runtime_error(msg); } while (0)

#endif

