#include "dma.h"

tal_err_t *dma_init(void) {
    return (void*) (0);
}

bool dma_configure_stream(const dma_config_t* config) {
    return 1;
}
inline static bool check_periph_dma_config_validity(periph_dma_config_t *dma_config) {
    return 1;
}

bool dma_start_transfer( dma_transfer_t *dma_transfer) {
    return 1;
}