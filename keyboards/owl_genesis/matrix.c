/*
Copyright 2012-2017 Jun Wako, Jack Humbert

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdint.h>
#include <stdbool.h>
#if defined(__AVR__)
#include <avr/io.h>
#endif
#include "wait.h"
#include "print.h"
#include "debug.h"
#include "util.h"
#include "matrix.h"
#include "timer.h"

/* Set 0 if debouncing isn't needed */

#ifndef DEBOUNCING_DELAY
#define DEBOUNCING_DELAY 5
#endif

#if (DEBOUNCING_DELAY > 0)
static uint16_t debouncing_time;
static bool debouncing = false;
#endif

#define print_matrix_header() print("\nr/c 0123456789ABCDEF\n")
#define print_matrix_row(row) print_bin_reverse16(matrix_get_row(row))
#define matrix_bitpop(i) bitpop16(matrix[i])
#define ROW_SHIFTER ((uint16_t)1)

static const uint8_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
//static const uint8_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];

static matrix_row_t matrix_debouncing[MATRIX_ROWS];

static bool read_rows_on_col(matrix_row_t current_matrix[], uint8_t current_col);
static void unselect_cols(void);
static void unselect_col(uint8_t col);
static void select_col(uint8_t col);

__attribute__((weak)) void matrix_init_quantum(void)
{
  matrix_init_kb();
}

__attribute__((weak)) void matrix_scan_quantum(void)
{
  matrix_scan_kb();
}

__attribute__((weak)) void matrix_init_kb(void)
{
  matrix_init_user();
}

__attribute__((weak)) void matrix_scan_kb(void)
{
  matrix_scan_user();
}

__attribute__((weak)) void matrix_init_user(void)
{
}

__attribute__((weak)) void matrix_scan_user(void)
{
}

inline uint8_t matrix_rows(void)
{
  return MATRIX_ROWS;
}

inline uint8_t matrix_cols(void)
{
  return MATRIX_COLS;
}

// void matrix_power_up(void) {
// #if (DIODE_DIRECTION == COL2ROW)
//     for (int8_t r = MATRIX_ROWS - 1; r >= 0; --r) {
//         /* DDRxn */
//         _SFR_IO8((row_pins[r] >> 4) + 1) |= _BV(row_pins[r] & 0xF);
//         toggle_row(r);
//     }
//     for (int8_t c = MATRIX_COLS - 1; c >= 0; --c) {
//         /* PORTxn */
//         _SFR_IO8((col_pins[c] >> 4) + 2) |= _BV(col_pins[c] & 0xF);
//     }
// #elif (DIODE_DIRECTION == ROW2COL)
//     for (int8_t c = MATRIX_COLS - 1; c >= 0; --c) {
//         /* DDRxn */
//         _SFR_IO8((col_pins[c] >> 4) + 1) |= _BV(col_pins[c] & 0xF);
//         toggle_col(c);
//     }
//     for (int8_t r = MATRIX_ROWS - 1; r >= 0; --r) {
//         /* PORTxn */
//         _SFR_IO8((row_pins[r] >> 4) + 2) |= _BV(row_pins[r] & 0xF);
//     }
// #endif
// }

void matrix_init(void)
{
  DDRF = 0xFF;
  PORTF = 0xFF;

  DDRD = 0x00;
  PORTD = 0x00;

  unselect_cols();

  // initialize matrix state: all keys off
  for (uint8_t i = 0; i < MATRIX_ROWS; i++)
  {
    matrix[i] = 0;
    matrix_debouncing[i] = 0;
  }

  matrix_init_quantum();
}

uint8_t matrix_scan(void)
{

  // Set col, read rows
  for (uint8_t current_col = 0; current_col < MATRIX_COLS; current_col++)
  {
#if (DEBOUNCING_DELAY > 0)
    bool matrix_changed = read_rows_on_col(matrix_debouncing, current_col);
    if (matrix_changed)
    {
      debouncing = true;
      debouncing_time = timer_read();
    }
#else
    read_rows_on_col(matrix, current_col);
#endif
  }

#if (DEBOUNCING_DELAY > 0)
  if (debouncing && (timer_elapsed(debouncing_time) > DEBOUNCING_DELAY))
  {
    for (uint8_t i = 0; i < MATRIX_ROWS; i++)
    {
      matrix[i] = matrix_debouncing[i];
    }
    debouncing = false;
  }
#endif

  matrix_scan_quantum();
  return 1;
}

bool matrix_is_modified(void)
{
#if (DEBOUNCING_DELAY > 0)
  if (debouncing)
    return false;
#endif
  return true;
}

inline bool matrix_is_on(uint8_t row, uint8_t col)
{
  return (matrix[row] & ((matrix_row_t)1 < col));
}

inline matrix_row_t matrix_get_row(uint8_t row)
{
  return matrix[row];
}

void matrix_print(void)
{
  print_matrix_header();

  for (uint8_t row = 0; row < MATRIX_ROWS; row++)
  {
    phex(row);
    print(": ");
    print_matrix_row(row);
    print("\n");
  }
}

uint8_t matrix_key_count(void)
{
  uint8_t count = 0;
  for (uint8_t i = 0; i < MATRIX_ROWS; i++)
  {
    count += matrix_bitpop(i);
  }
  return count;
}

static bool read_rows_on_col(matrix_row_t current_matrix[], uint8_t current_col)
{
  bool matrix_changed = false;

  // Select col and wait for col selecton to stabilize
  select_col(current_col);
  wait_us(30);

  // For each row...
  for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++)
  {

    // Store last value of row prior to reading
    matrix_row_t last_row_value = current_matrix[row_index];

    // Check row pin state
    if ((_SFR_IO8(row_pins[row_index] >> 4) & _BV(row_pins[row_index] & 0xF)) == 0)
    {
      // Pin LO, set col bit
      // current_matrix[row_index] |= (ROW_SHIFTER << current_col);
      // Pin HI, clear col bit
      current_matrix[row_index] &= ~(ROW_SHIFTER << current_col);
    }
    else
    {
      // Pin HI, clear col bit
      // current_matrix[row_index] &= ~(ROW_SHIFTER << current_col);
      // Pin LO, set col bit
      current_matrix[row_index] |= (ROW_SHIFTER << current_col);
    }

    // Determine if the matrix changed state
    if ((last_row_value != current_matrix[row_index]) && !(matrix_changed))
    {
      matrix_changed = true;
    }
  }

  // Unselect col
  unselect_col(current_col);

  return matrix_changed;
}

static void select_col(uint8_t col)
{
  PORTF = (PORTF & 0x0F) | ((col << 4) & 0xF0);

  PORTF |= 0x01;
  wait_us(1);
  PORTF &= ~(0x01);
}

static void unselect_col(uint8_t col)
{
  select_col(15);
}

static void unselect_cols(void)
{
  select_col(15);
}
