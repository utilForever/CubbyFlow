// This code is based on Jet framework.
// Copyright (c) 2018 Doyub Kim
// CubbyFlow is voxel-based fluid simulation engine for computer games.
// Copyright (c) 2020 CubbyFlow Team
// Core Part: Chris Ohk, Junwoo Hwang, Jihong Sin, Seungwoo Yoo
// AI Part: Dongheon Cho, Minseo Kim
// We are making my contributions/submissions to this project solely in our
// personal capacity and are not conveying any rights to any intellectual
// property of any third parties.

#ifndef CUBBYFLOW_SERIALIZATION_IMPL_HPP
#define CUBBYFLOW_SERIALIZATION_IMPL_HPP

#include <cstring>
#include <stdexcept>
#include <type_traits>

namespace CubbyFlow
{
template <typename T>
void Serialize(const ConstArrayView1<T>& array, std::vector<uint8_t>* buffer)
{
    static_assert(std::is_trivially_copyable_v<T>);

    const size_t size = sizeof(T) * array.Length();
    Serialize(reinterpret_cast<const uint8_t*>(array.data()), size, buffer);
}

template <typename T>
void Deserialize(const std::vector<uint8_t>& buffer, Array1<T>* array)
{
    static_assert(std::is_trivially_copyable_v<T>);

    std::vector<uint8_t> data;
    Deserialize(buffer, &data);

    if (data.size() % sizeof(T) != 0)
    {
        throw std::invalid_argument{
            "Serialized data size must be a multiple of the element size."
        };
    }

    array->Resize(data.size() / sizeof(T));
    std::memcpy(array->data(), data.data(), data.size());
}
}  // namespace CubbyFlow

#endif
