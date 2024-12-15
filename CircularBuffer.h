#pragma once
#include <vector>

template <typename T>
class CircularBuffer
{
public:

	inline CircularBuffer<T>() {
		this->data = std::vector<T>(0);
	}
	inline CircularBuffer<T>(int _maxSize) {
		this->data = std::vector<T>(0);
		this->maxElements = _maxSize;
	}

	T& operator[](int _index) {
		return data[_index % data.size()];
	}

	void push(T _input) {
		data.push_back(_input);
		this->resize();
	}
	void push(std::vector<T> _inputs) {
		for (T item : _inputs) {
			data.push_back(item);
		}
		this->resize();
	}

	void maxSize(int _maxSize) {
		this->maxElements = _maxSize;
	}
	size_t size() {
		return this->data.size();
	}

private: 
	void resize() {
		while (data.size() > maxElements && data.size()) {
			data.erase(data.begin());
		}
	}
	std::vector<T> data;
	int maxElements = 0;
};
