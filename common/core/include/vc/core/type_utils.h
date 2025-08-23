#pragma once

#include <vector>
#include <list>
#include <deque>
#include <memory>

template <typename T>
using shared_const_ptr = std::shared_ptr<const T>;

// vector 版本
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::vector<shared_const_ptr<T>> to_const(const std::vector<std::shared_ptr<T>, Alloc> &c)
{
    return std::vector<shared_const_ptr<T>>(c.begin(), c.end());
}

// list 版本
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::list<shared_const_ptr<T>> to_const(const std::list<std::shared_ptr<T>, Alloc> &c)
{
    return std::list<shared_const_ptr<T>>(c.begin(), c.end());
}

// deque 版本
template <typename T, typename Alloc = std::allocator<std::shared_ptr<T>>>
std::deque<shared_const_ptr<T>> to_const(const std::deque<std::shared_ptr<T>, Alloc> &c)
{
    return std::deque<shared_const_ptr<T>>(c.begin(), c.end());
}

// tuple 版本：将 tuple<shared_ptr<T>...> 转换为 tuple<shared_ptr<const T>...>
template <typename... Ts>
auto to_const(const std::tuple<std::shared_ptr<Ts>...> &tpl)
{
    return std::apply([](auto const &...elems)
                      { return std::make_tuple(shared_const_ptr<Ts>(elems)...); }, tpl);
}

// --- vector<tuple<...>> 版本 ---
template <typename... Ts>
auto to_const(const std::vector<std::tuple<std::shared_ptr<Ts>...>> &vec)
{
    std::vector<std::tuple<shared_const_ptr<Ts>...>> result;
    result.reserve(vec.size());

    for (auto const &t : vec)
    {
        result.push_back(to_const(t));
    }
    return result;
}