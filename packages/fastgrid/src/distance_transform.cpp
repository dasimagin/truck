#include "fastgrid/distance_transform.h"

#include <array>
#include <limits>

namespace truck::fastgrid {

namespace impl {

// DT with BORDER_SIZE = n requires 2 distance constants if n == 1 and 2 * n - 1 distance constants
// otherwise, sorted in distance ascending order
template<int BORDER_SIZE>
struct Neighbourhood {
    constexpr Neighbourhood(std::array<int32_t, 2 * BORDER_SIZE + 1 / BORDER_SIZE - 1>&& arr)
        : distance(arr), max_value(std::numeric_limits<int32_t>::max() - arr.back()) {}

    const std::array<int32_t, 2 * BORDER_SIZE + 1 / BORDER_SIZE - 1> distance;
    const int32_t max_value;
};

constexpr Neighbourhood<1> neighbourhood3 = {{3, 4}};
constexpr Neighbourhood<2> neighbourhood5 = {{5, 7, 11}};

template<int BORDER_SIZE, Neighbourhood<BORDER_SIZE> neighbourhood>
inline int32_t RowMin(const int row, const int32_t* row_mid_ptr) noexcept {
    int32_t result = neighbourhood.max_value;
    for (int i = 1; i < row; ++i) {
        result = std::min(
            result,
            std::min(row_mid_ptr[-i], row_mid_ptr[i]) + neighbourhood.distance[row + i - 1]);
    }
    for (int i = row + 1; i <= BORDER_SIZE; ++i) {
        result = std::min(
            result,
            std::min(row_mid_ptr[-i], row_mid_ptr[i]) + neighbourhood.distance[row + i - 1]);
    }
    return result;
}

template<int BORDER_SIZE, Neighbourhood<BORDER_SIZE> neighbourhood>
inline int32_t UpperNeighbourhoodMin(const int32_t* buf_ptr, const Size& buf_size) noexcept {
    const int32_t* row_mid_ptr = buf_ptr - buf_size.width;
    int32_t result = std::min(buf_ptr[-1], *row_mid_ptr) + neighbourhood.distance[0];
    result =
        std::min(result, std::min(row_mid_ptr[-1], row_mid_ptr[1]) + neighbourhood.distance[1]);
    for (int row = 1; row <= BORDER_SIZE; ++row, row_mid_ptr -= buf_size.width) {
        result = std::min(result, RowMin<BORDER_SIZE, neighbourhood>(row, row_mid_ptr));
    }
    return std::min(result, neighbourhood.max_value);
}

template<int BORDER_SIZE, Neighbourhood<BORDER_SIZE> neighbourhood>
inline int32_t LowerNeighbourhoodMin(const int32_t* buf_ptr, const Size& buf_size) noexcept {
    const int32_t* row_mid_ptr = buf_ptr + buf_size.width;
    int32_t result = std::min(buf_ptr[1], *row_mid_ptr) + neighbourhood.distance[0];
    result =
        std::min(result, std::min(row_mid_ptr[-1], row_mid_ptr[1]) + neighbourhood.distance[1]);
    for (int row = 1; row <= BORDER_SIZE; ++row, row_mid_ptr += buf_size.width) {
        result = std::min(result, RowMin<BORDER_SIZE, neighbourhood>(row, row_mid_ptr));
    }
    return std::min(result, neighbourhood.max_value);
}

template<int BORDER_SIZE, Neighbourhood<BORDER_SIZE> neighbourhood>
inline void DistanceTransformApprox(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    const float scale = static_cast<float>(input.resolution) / neighbourhood.distance[0];

    int32_t* buf_ptr = buf.data;
    int32_t* buf_ptr_rev = buf.data + buf.size() - 1;
    for (int i = 0; i < BORDER_SIZE * buf.size.width; ++i, ++buf_ptr, --buf_ptr_rev) {
        *buf_ptr = neighbourhood.max_value;
        *buf_ptr_rev = neighbourhood.max_value;
    }

    uint8_t* input_ptr = input.data;
    for (int i = 0; i < input.size.height; ++i, buf_ptr += BORDER_SIZE) {
        int32_t* row_end = buf_ptr + buf.size.width - 1;
        for (int j = 0; j < BORDER_SIZE; ++j, ++buf_ptr, --row_end) {
            *buf_ptr = neighbourhood.max_value;
            *row_end = neighbourhood.max_value;
        }
        ++row_end;
        for (; buf_ptr != row_end; ++buf_ptr, ++input_ptr) {
            if (*input_ptr == 0) {
                *buf_ptr = 0;
                continue;
            }
            *buf_ptr = UpperNeighbourhoodMin<BORDER_SIZE, neighbourhood>(buf_ptr, buf.size);
        }
    }

    float* out_ptr = out.data + out.size() - 1;
    buf_ptr_rev -= BORDER_SIZE;
    for (int i = 0; i < input.size.height; ++i, buf_ptr_rev -= 2 * BORDER_SIZE) {
        for (int j = 0; j < input.size.width; ++j, --buf_ptr_rev, --out_ptr) {
            *buf_ptr_rev = std::min(
                *buf_ptr_rev,
                LowerNeighbourhoodMin<BORDER_SIZE, neighbourhood>(buf_ptr_rev, buf.size));
            *out_ptr = scale * (*buf_ptr_rev);
        }
    }
}

template<int BORDER_SIZE, Neighbourhood<BORDER_SIZE> neighbourhood>
inline void DistanceTransformApprox(const U8Grid& input, F32Grid& out) noexcept {
    S32GridHolder buf = MakeGrid<int32_t>(
        Size(
            {.width = input.size.width + 2 * BORDER_SIZE,
             .height = input.size.height + 2 * BORDER_SIZE}),
        input.resolution);
    DistanceTransformApprox<BORDER_SIZE, neighbourhood>(input, *buf, out);
}

template<int BORDER_SIZE, Neighbourhood<BORDER_SIZE> neighbourhood>
inline F32GridHolder DistanceTransformApprox(const U8Grid& input) noexcept {
    F32GridHolder out = MakeGridLike<float>(input);
    DistanceTransformApprox<BORDER_SIZE, neighbourhood>(input, *out);
    return out;
}

}  // namespace impl

/* DistanceTransformApprox3 */

void DistanceTransformApprox3(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    impl::DistanceTransformApprox<1, impl::neighbourhood3>(input, buf, out);
}

void DistanceTransformApprox3(const U8Grid& input, F32Grid& out) noexcept {
    impl::DistanceTransformApprox<1, impl::neighbourhood3>(input, out);
}

F32GridHolder DistanceTransformApprox3(const U8Grid& input) noexcept {
    return impl::DistanceTransformApprox<1, impl::neighbourhood3>(input);
}

/* DistanceTransformApprox5 */

void DistanceTransformApprox5(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    impl::DistanceTransformApprox<2, impl::neighbourhood5>(input, buf, out);
}

void DistanceTransformApprox5(const U8Grid& input, F32Grid& out) noexcept {
    impl::DistanceTransformApprox<2, impl::neighbourhood5>(input, out);
}

F32GridHolder DistanceTransformApprox5(const U8Grid& input) noexcept {
    return impl::DistanceTransformApprox<2, impl::neighbourhood5>(input);
}

}  // namespace truck::fastgrid
