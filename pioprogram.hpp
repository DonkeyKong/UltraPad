#include <pico/stdlib.h>
#include <hardware/pio.h>
#include <hardware/timer.h>

#include <iostream>

class PioProgram
{
public:
  
  PioProgram() = default;
  
  PioProgram(const PIO& pio, const pio_program* prog, pio_sm_config (*configFunc)(PIO, uint, uint, uint, float), uint pin, float clkdiv = 16.625f)
  {
    // Load and start the N64 controller PIO program
    pio_ = pio;
    prog_ = prog;
    offset_ = pio_add_program(pio_, prog_);
    sm_ = pio_claim_unused_sm(pio_, true);
    config_ = configFunc(pio_, sm_, offset_, pin, clkdiv);
    loaded_ = true;
  }

  ~PioProgram()
  {
    if (loaded_)
    {
      pio_sm_unclaim(pio_, sm_);
      pio_remove_program(pio_, prog_, offset_);
    }
    loaded_ = false;
  }

  PioProgram(const PioProgram& o) = delete;

  PioProgram(PioProgram&& other):
    pio_(other.pio_),
    offset_(other.offset_),
    sm_(other.sm_),
    prog_(other.prog_),
    config_(other.config_),
    loaded_(other.loaded_)
  {
    other.loaded_ = false;
  }

  PioProgram& operator=(PioProgram&& other)
  {
    if (loaded_)
    {
      pio_sm_unclaim(pio_, sm_);
      pio_remove_program(pio_, prog_, offset_);
    }
    pio_ = other.pio_;
    offset_ = other.offset_;
    sm_ = other.sm_;
    prog_ = other.prog_;
    config_ = other.config_;
    loaded_ = other.loaded_;
    other.loaded_ = false;
    return *this;
  }

  bool writeLengthAndBytes(uint32_t data, uint32_t bytesToSend, int timeoutMs = -1)
  {
    auto start = to_ms_since_boot(get_absolute_time());
    while (pio_sm_is_tx_fifo_full(pio_, sm_)) 
    {  
      auto now = to_ms_since_boot(get_absolute_time());
      if (timeoutMs > 0 && (now - start) > timeoutMs)
      {
        return false;
      }
    }
    pio_sm_put(pio_, sm_, bytesToSend*8-1);
    while (pio_sm_is_tx_fifo_full(pio_, sm_)) 
    {  
      auto now = to_ms_since_boot(get_absolute_time());
      if (timeoutMs > 0 && (now - start) > timeoutMs)
      {
        return false;
      }
    }
    pio_sm_put(pio_, sm_, data);
    return true;
  }

  bool write(uint32_t data, int timeoutMs = -1)
  {
    auto start = to_ms_since_boot(get_absolute_time());
    while (pio_sm_is_tx_fifo_full(pio_, sm_)) 
    {  
      auto now = to_ms_since_boot(get_absolute_time());
      if (timeoutMs > 0 && (now - start) > timeoutMs)
      {
        return false;
      }
    }
    pio_sm_put(pio_, sm_, data);
    return true;
  }

  bool write(uint8_t data, int timeoutMs = -1)
  {
    return write((uint32_t)data << 24, timeoutMs);
  }

  bool read(uint32_t& data, int timeoutMs = -1)
  {
    auto start = to_ms_since_boot(get_absolute_time());
    while (pio_sm_is_rx_fifo_empty(pio_, sm_)) 
    {  
      auto now = to_ms_since_boot(get_absolute_time());
      if (timeoutMs > 0 && (now - start) > timeoutMs)
      {
        return false;
      }
    }
    data = pio_sm_get(pio_, sm_);
    return true;
  }

  void reset()
  {
    pio_sm_set_enabled(pio_, sm_, false);
    pio_sm_clear_fifos(pio_, sm_);
    pio_sm_restart(pio_, sm_);
    pio_sm_init(pio_, sm_, offset_, &config_);
    pio_sm_set_enabled(pio_, sm_, true);
  }

private:
  PIO pio_;
  uint offset_;
  uint sm_;
  const pio_program* prog_;
  pio_sm_config config_;
  bool loaded_ = false;
};
