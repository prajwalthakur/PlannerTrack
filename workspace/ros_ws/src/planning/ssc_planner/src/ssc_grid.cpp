/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 */
#include "ssc_planner/ssc_grid.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace ssc_planner
{

//////////////////////////////////////////////////////////////////////////

GridMapMetaInfo::GridMapMetaInfo()
{
}

//////////////////////////////////////////////////////////////////////////

GridMapMetaInfo::GridMapMetaInfo(const int w, const int h, const double res)
    : width(w), height(h), resolution(res)
{
	w_metric = w * resolution;
	h_metric = h * resolution;
}

//////////////////////////////////////////////////////////////////////////

void GridMapMetaInfo::print() const
{
	printf("GridMapMetaInfo:\n");
	printf(" -- width:%d\n", width);
	printf(" -- height:%d\n", height);
	printf(" -- resolution:%lf\n", resolution);
	printf(" -- w_metric:%lf\n", w_metric);
	printf(" -- h_metric:%lf\n", h_metric);
	printf("\n");
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND()
{
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(const std::array<int, N_DIM> & dims_size,
    const std::array<float, N_DIM> & dims_resolution,
    const std::array<std::string, N_DIM> & dims_name)
{
	dims_size_ = dims_size;
	dims_resolution_ = dims_resolution;
	dims_name_ = dims_name;

	SetNDimSteps(dims_size_);
	SetDataSize(dims_size_);
	data_ = std::vector<T>(data_size_, 0);
	origin_.fill(0);
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingCoordinate(
    const std::array<int, N_DIM> & coord, T * val) const
{
	if (!CheckCoordInRange(coord)) {
		// printf("[GridMapND] Out of range\n");
		return kWrongStatus;
	}
	int idx = GetMonoIdxUsingNDimIdx(coord);
	*val = data_[idx];
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRange(const std::array<int, N_DIM> & coord) const
{
	for (int i = 0; i < N_DIM; ++i) {
		if (coord[i] < 0 || coord[i] >= dims_size_[i]) {
			return false;
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingGlobalPosition(
    const std::array<float, N_DIM> & p_w, T * val) const
{
	std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
	GetValueUsingCoordinate(coord, val);
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetCoordUsingGlobalPosition(
    const std::array<float, N_DIM> & p_w) const
{
	std::array<int, N_DIM> coord = {};
	for (int i = 0; i < N_DIM; ++i) {
		coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
	}
	return coord;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingGlobalPosition(
    const std::array<float, N_DIM> & p_w, const T & val_in, bool * res) const
{
	std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
	T val;
	if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
		*res = false;
	} else {
		*res = (val == val_in);
	}
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingCoordinate(
    const std::array<int, N_DIM> & coord, const T & val_in, bool * res) const
{
	T val;
	if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
		*res = false;
	} else {
		*res = (val == val_in);
	}
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
std::array<float, N_DIM> GridMapND<T, N_DIM>::GetRoundedPosUsingGlobalPosition(
    const std::array<float, N_DIM> & p_w) const
{
	// grid coordinates
	std::array<int, N_DIM> coord = {};
	for (int i = 0; i < N_DIM; ++i) {
		coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
	}
	// back to map coordinates,quantized to the grid's resolution.
	std::array<float, N_DIM> round_pos = {};
	for (int i = 0; i < N_DIM; ++i) {
		round_pos[i] = coord[i] * dims_resolution_[i] + origin_[i];
	}
	return round_pos;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalPositionUsingCoordinate(
    const std::array<int, N_DIM> & coord, std::array<float, N_DIM> * p_w) const
{
	auto ptr = p_w->data();
	for (int i = 0; i < N_DIM; ++i) {
		*(ptr + i) = coord[i] * dims_resolution_[i] + origin_[i];
	}
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////
/**
 * 
 */
template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetCoordUsingGlobalMetricOnSingleDim(
    const float & metric, const int & i, int * idx) const
{
	*idx = std::round((metric - origin_[i]) / dims_resolution_[i]);
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalMetricUsingCoordOnSingleDim(
    const int & idx, const int & i, float * metric) const
{
	*metric = idx * dims_resolution_[i] + origin_[i];
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRangeOnSingleDim(const int & idx, const int & i) const
{
	return (idx >= 0) && (idx < dims_size_[i]);
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingCoordinate(
    const std::array<int, N_DIM> & coord, const T & val)
{
	if (!CheckCoordInRange(coord)) {
		// printf("[GridMapND] Out of range\n");
		return kWrongStatus;
	}
	int idx = GetMonoIdxUsingNDimIdx(coord);
	data_[idx] = val;
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
int GridMapND<T, N_DIM>::GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> & idx) const
{
	int mono_idx = 0;
	for (int i = 0; i < N_DIM; ++i) {
		mono_idx += dims_step_[i] * idx[i];
	}
	return mono_idx;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetNDimIdxUsingMonoIdx(const int & idx) const
{
	std::array<int, N_DIM> idx_nd = {};
	int tmp = idx;
	for (int i = N_DIM - 1; i >= 0; --i) {
		idx_nd[i] = tmp / dims_step_[i];
		tmp = tmp % dims_step_[i];
	}
	return idx_nd;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingGlobalPosition(
    const std::array<float, N_DIM> & p_w, const T & val)
{
	std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
	SetValueUsingCoordinate(coord, val);
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetNDimSteps(const std::array<int, N_DIM> & dims_size)
{
	int step = 1;
	for (int i = 0; i < N_DIM; ++i) {
		dims_step_[i] = step;
		step = step * dims_size[i];
	}
	return kSuccess;
}

//////////////////////////////////////////////////////////////////////////

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetDataSize(const std::array<int, N_DIM> & dims_size)
{
	int total_ele_num = 1;
	for (int i = 0; i < N_DIM; ++i) {
		total_ele_num = total_ele_num * dims_size_[i];
	}
	data_size_ = total_ele_num;
	return kSuccess;
}
template class GridMapND<uint8_t, 2>;
template class GridMapND<uint8_t, 3>;
template class GridMapND<int, 2>;
template class GridMapND<int, 3>;

}  // namespace ssc_planner