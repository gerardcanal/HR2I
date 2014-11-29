#pragma once
#include <deque>
#include <vector>

template <class _T>
class SlidingMatrix
{
public:
	SlidingMatrix(int rows, int columns);
	SlidingMatrix(int rows, int columns, _T init);
	~SlidingMatrix();

	void slide();
	int size() const;
	int rows() const;
	int columns() const;

	std::vector<_T>& operator[](int i);
	const std::vector<_T>& operator[](int i) const;
	_T& operator()(int row, int column); // To acces by m(row, column) instead of m(column, row);
	const _T& operator()(int row, int column) const; // To acces by m(row, column) instead of m(column, row);

	friend std::ostream &operator<<(std::ostream& os, const SlidingMatrix<_T>& m)  {
		for (int i = 0; i < m[0].size(); ++i) { // Rows
			for (int j = 0; j < m.size(); ++j) { // Columns
				os << m[j][i] << ((j + 1 == m.size())? "\n" : " ");
			}
		}
		return os;
	}
private:
	std::deque<std::vector<_T>*> matrix;
};

template <class _T>
std::vector<_T>& SlidingMatrix<_T>::operator[](int i) { 
	return *matrix[i]; 
}

template <class _T>
const std::vector<_T>& SlidingMatrix<_T>::operator[](int i) const { 
	return *matrix[i]; 
}

template <class _T>
_T& SlidingMatrix<_T>::operator()(int row, int column) {
	return (*matrix[column])[row]; 
}

template<class _T>
const _T& SlidingMatrix<_T>::operator()(int row, int column) const {
	return (*matrix[column])[row];
}

template <class _T>
SlidingMatrix<_T>::SlidingMatrix(int rows, int columns)
{
	matrix = std::deque<std::vector<_T>*>(columns);
	for (std::deque<std::vector<_T>*>::iterator it = matrix.begin(); it != matrix.end(); ++it) {
		*it = new std::vector<_T>(rows);
	}
}

template <class _T>
SlidingMatrix<_T>::SlidingMatrix(int rows, int columns, _T value)
{
	matrix = std::deque<std::vector<_T>*>(columns);
	for (std::deque<std::vector<_T>*>::iterator it = matrix.begin(); it != matrix.end(); ++it) {
		*it = new std::vector<_T>(rows, value);
	}
}

template <class _T>
SlidingMatrix<_T>::~SlidingMatrix()
{
	for (std::deque<std::vector<_T>*>::iterator it = matrix.begin(); it != matrix.end(); ++it) {
		delete *it;
	}
}

template <class _T>
void SlidingMatrix<_T>::slide() {
	std::vector<_T>* aux = matrix.front(); // Get first row pointer
	matrix.pop_front(); // Remove first row pointer
	matrix.push_back(aux); // Add the element at the back
}

template <class _T>
int SlidingMatrix<_T>::size() const { // Returns the number of columns!!!
	return matrix.size();
}

template <class _T>
int SlidingMatrix<_T>::rows() const {
	if (matrix.size() == 0) return 0;
	return matrix[0]->size();
}

template <class _T>
int SlidingMatrix<_T>::columns() const {
	return matrix.size();
}