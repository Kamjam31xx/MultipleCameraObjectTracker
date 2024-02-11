#pragma once

#include <iostream>

using Entity = std::uint64_t;
const Entity MAX_ENTITIES = 4096;
using ComponentType = std::uint64_t;
const ComponentType MAX_COMPONENTS = 64;
using Signature = std::bitset<MAX_COMPONENTS>;



