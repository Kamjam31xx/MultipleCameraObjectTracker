#pragma once
#include <iostream>
#include <queue>
#include <bitset>
#include <array>
#include <assert.h>
#include "EntityValues.h"

class EntityManager
{
public:
	EntityManager();
	Entity createEntity();
	void destroyEntity(Entity entity);
	void setSignature(Entity entity, Signature signature);
	Signature getSignature(Entity entity);


private: 
	std::queue<Entity> availableEntities{};
	std::array<Signature, MAX_ENTITIES> signatures{};
	uint64_t livingEntityCount{};
};

