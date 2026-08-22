/*
 * Author: Prajwal Thakur <prajwalthakur98@gmail.com>
 * Adapted from Epsilon code
 */
#pragma once

#include <array>
#include <string>
#include <vector>

namespace ssc_planner
{

enum ErrorType { kSuccess = 0, kWrongStatus, kIllegalInput, kUnknown };

struct GridMapMetaInfo
{
	int width = 0;
	int height = 0;
	double resolution = 0;
	double w_metric = 0;
	double h_metric = 0;

	/**
	 * @brief Construct a new Grid Map Meta Info object
	 */
	GridMapMetaInfo();

	/**
	 * @brief Construct a new Grid Map Meta Info object
	 *
	 * @param w width
	 * @param h height
	 * @param res resolution
	 */
	GridMapMetaInfo(const int w, const int h, const double res);

	/**
	 * @brief Print info
	 */
	void print() const;
};

/**
 * @brief Occupancy grid map
 *
 * @tparam T Data type of the grid map
 * @tparam N_DIM Dimension of the grid map
 */
template <typename T, int N_DIM>
class GridMapND
{
  public:
	enum ValType {
		OCCUPIED = 70,
		// FREE = 102,
		FREE = 0,
		SCANNED_OCCUPIED = 128,
		UNKNOWN = 0
	};
	/**
	 * @brief Construct a new GridMapND object
	 *
	 */
	GridMapND();
	/**
	 * @brief Construct a new GridMapND object
	 *
	 * @param dims_size size of each dimension
	 * @param dims_resolution resolution of each dimension
	 * @param dims_name name of each dimension
	 */
	GridMapND(const std::array<int, N_DIM> & dims_size,
	    const std::array<float, N_DIM> & dims_resolution,
	    const std::array<std::string, N_DIM> & dims_name);

	inline std::array<int, N_DIM> dims_size() const { return dims_size_; }

	inline int dims_size(const int & dim) const { return dims_size_.at(dim); }

	inline std::array<int, N_DIM> dims_step() const { return dims_step_; }

	inline int dims_step(const int & dim) const { return dims_step_.at(dim); }

	inline std::array<float, N_DIM> dims_resolution() const { return dims_resolution_; }

	inline float dims_resolution(const int & dim) const { return dims_resolution_.at(dim); }

	inline std::array<std::string, N_DIM> dims_name() const { return dims_name_; }

	inline std::string dims_name(const int & dim) const { return dims_name_.at(dim); }

	inline std::array<float, N_DIM> origin() const { return origin_; }

	inline int data_size() const { return data_size_; }

	inline const std::vector<T> * data() const { return &data_; }

	inline T data(const int & i) const { return data_[i]; };

	inline T * get_data_ptr() { return data_.data(); }

	inline const T * data_ptr() const { return data_.data(); }

	inline void set_origin(const std::array<float, N_DIM> & origin) { origin_ = origin; }

	inline void set_dims_size(const std::array<int, N_DIM> & dims_size)
	{
		dims_size_ = dims_size;
		SetNDimSteps(dims_size);
		SetDataSize(dims_size);
	}
	inline void set_dims_resolution(const std::array<float, N_DIM> & dims_resolution)
	{
		dims_resolution_ = dims_resolution;
	}
	inline void set_dims_name(const std::array<std::string, N_DIM> & dims_name)
	{
		dims_name_ = dims_name;
	}
	inline void set_data(const std::vector<T> & in) { data_ = in; }

	/**
	 * @brief Set all data in array to 0
	 */
	inline void clear_data() { data_ = std::vector<T>(data_size_, 0); }

	/**
	 * @brief Fill all data in array using value
	 *
	 * @param val input value
	 */
	inline void fill_data(const T & val) { data_ = std::vector<T>(data_size_, val); }

	/**
	 * @brief Get the Value Using Coordinate
	 *
	 * @param coord Input coordinate
	 * @param val Output value
	 * @return ErrorType
	 */
	ErrorType GetValueUsingCoordinate(const std::array<int, N_DIM> & coord, T * val) const;

	/**
	 * @brief Get the Value Using Global Position
	 *
	 * @param p_w Input global position
	 * @param val Output value
	 * @return ErrorType
	 */
	ErrorType GetValueUsingGlobalPosition(const std::array<float, N_DIM> & p_w, T * val) const;

	/**
	 * @brief Check if the input value is equal to the value in map
	 *
	 * @param p_w Input global position
	 * @param val_in Input value
	 * @param res Result
	 * @return ErrorType
	 */
	ErrorType CheckIfEqualUsingGlobalPosition(
	    const std::array<float, N_DIM> & p_w, const T & val_in, bool * res) const;

	/**
	 * @brief Check if the input value is equal to the value in map
	 *
	 * @param coord Input coordinate
	 * @param val_in Input value
	 * @param res Result
	 * @return ErrorType
	 */
	ErrorType CheckIfEqualUsingCoordinate(
	    const std::array<int, N_DIM> & coord, const T & val_in, bool * res) const;

	/**
	 * @brief Set the Value Using Coordinate
	 *
	 * @param coord Coordinate of the map
	 * @param val Input value
	 * @return ErrorType
	 */
	ErrorType SetValueUsingCoordinate(const std::array<int, N_DIM> & coord, const T & val);

	/**
	 * @brief Set the Value Using Global Position
	 *
	 * @param p_w Global position
	 * @param val Input value
	 * @return ErrorType
	 */
	ErrorType SetValueUsingGlobalPosition(const std::array<float, N_DIM> & p_w, const T & val);

	/**
	 * @brief Get the Coordinate Using Global Position
	 *
	 * @param p_w Input global position
	 * @return std::array<int, N_DIM> Output coordinate
	 */
	std::array<int, N_DIM> GetCoordUsingGlobalPosition(const std::array<float, N_DIM> & p_w) const;

	/**
	 * @brief Get the Rounded Position Using Global Position object
	 *
	 * @param p_w Input global position
	 * @return std::array<float, N_DIM> Output global position
	 */
	std::array<float, N_DIM> GetRoundedPosUsingGlobalPosition(
	    const std::array<float, N_DIM> & p_w) const;

	/**
	 * @brief Get the Global Position Using Coordinate
	 *
	 * @param coord Input coordinate
	 * @param p_w Output global position
	 * @return ErrorType
	 */
	ErrorType GetGlobalPositionUsingCoordinate(
	    const std::array<int, N_DIM> & coord, std::array<float, N_DIM> * p_w) const;

	/**
	 * @brief Get the Coordinate Using Global Metric On Single Dimension
	 *
	 * @param metric Input global 1-dim position
	 * @param i Dimension
	 * @param idx Output 1-d coordinate
	 * @return ErrorType
	 */
	ErrorType GetCoordUsingGlobalMetricOnSingleDim(
	    const float & metric, const int & i, int * idx) const;

	/**
	 * @brief Get the Global Metric Using Coordinate On Single Dim object
	 *
	 * @param idx Input 1-d coordinate
	 * @param i Dimension
	 * @param metric Output 1-d position
	 * @return ErrorType
	 */
	ErrorType GetGlobalMetricUsingCoordOnSingleDim(
	    const int & idx, const int & i, float * metric) const;

	/**
	 * @brief Check if the input coordinate is in map range
	 *
	 * @param coord Input coordinate
	 * @return true In range
	 * @return false Out of range
	 */
	bool CheckCoordInRange(const std::array<int, N_DIM> & coord) const;

	/**
	 * @brief Check if the input 1-d coordinate is in map range
	 *
	 * @param idx Input 1-d coordinate
	 * @param i Dimension
	 * @return true In range
	 * @return false Out of range
	 */
	bool CheckCoordInRangeOnSingleDim(const int & idx, const int & i) const;

	/**
	 * @brief Get the mono index using N-dim index
	 *
	 * @param idx Input N-dim index
	 * @return int Output 1-dim index
	 */
	int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM> & idx) const;

	/**
	 * @brief Get N-dim index using mono index
	 *
	 * @param idx Input mono index
	 * @return std::array<int, N_DIM> Output N-dim index
	 */
	std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int & idx) const;

  private:
	/**
	 * @brief Set the steps of N-dim array
	 * @brief E.g. A x-y-z map's steps are {1, x, x*y}
	 *
	 * @param dims_size Input the size of dimension
	 * @return ErrorType
	 */
	ErrorType SetNDimSteps(const std::array<int, N_DIM> & dims_size);

	/**
	 * @brief Get the Data Size
	 *
	 * @param dims_size Input the size of dimension
	 * @return ErrorType
	 */
	ErrorType SetDataSize(const std::array<int, N_DIM> & dims_size);

	std::array<int, N_DIM> dims_size_;
	std::array<int, N_DIM> dims_step_;
	std::array<float, N_DIM> dims_resolution_;
	std::array<std::string, N_DIM> dims_name_;
	std::array<float, N_DIM> origin_;

	int data_size_{0};
	std::vector<T> data_;
};

}  // namespace ssc_planner