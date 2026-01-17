#include "sync/resampler.h"

namespace ridersense {

template class Resampler<ImuFrame>;
template class Resampler<GpsFrame>;

} // namespace ridersense
