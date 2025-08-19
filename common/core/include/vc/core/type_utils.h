#pragma once

#include <vector>
#include <list>
#include <deque>
#include <memory>

template <typename T>
using shared_const_ptr = std::shared_ptr<const T>;

// vector 版本
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::vector<shared_const_ptr<T>> to_const_ptr(const std::vector<std::shared_ptr<T>, Alloc> &c)
{
    return std::vector<shared_const_ptr<T>>(c.begin(), c.end());
}

// list 版本
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::list<shared_const_ptr<T>> to_const_ptr(const std::list<std::shared_ptr<T>, Alloc> &c)
{
    return std::list<shared_const_ptr<T>>(c.begin(), c.end());
}

// deque 版本
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::deque<shared_const_ptr<T>> to_const_ptr(const std::deque<std::shared_ptr<T>, Alloc> &c)
{
    return std::deque<shared_const_ptr<T>>(c.begin(), c.end());
}
