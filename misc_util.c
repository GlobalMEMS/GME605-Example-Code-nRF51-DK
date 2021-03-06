/*
 *
 ****************************************************************************
 * Copyright (C) 2016 GlobalMEMS, Inc. <www.globalmems.com>
 * All rights reserved.
 *
 * File : misc_util.c
 *
 * Usage: Miscellaneous utility Function
 *
 ****************************************************************************
 * 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 **************************************************************************/

/*! @file misc_util.c
 *  @brief Miscellaneous utility Function
 *  @author Joseph FC Tseng
 */

#include "misc_util.h"

/*!
 * @brief Rotate the axis according to layout pattern
 *
 * @param[in] pat Layout pattern number
 * @param[in/out] pi32Data pointer to raw int32_t data vectors
 *
 * @return 1 for Success
 * @return 0 for Error
 */
int16_t coord_rotate(const AKFS_PATNO pat, raw_data_xyzt_t *pi32Data)
{
  AKFVEC vecTmp;
  int i;
  int16_t res;

  for(i = 0; i < 3; ++i)
    vecTmp.v[i] = pi32Data->v[i];

  res = AKFS_Rotate(pat, &vecTmp);

  for(i = 0; i < 3; ++i)
    pi32Data->v[i] = (int32_t) vecTmp.v[i];

  return res;
}

/*!
 * @brief Rotate the axis according to layout pattern
 *
 * @param[in] pat Layout pattern number
 * @param[in/out] pfData pointer to raw float data vectors
 *
 * @return 1 for Success
 * @return 0 for Error
 */
int16_t coord_rotate_f(const AKFS_PATNO pat, float_xyzt_t *pfData)
{
  AKFVEC vecTmp;
  int i;
  int16_t res;

  for(i = 0; i < 3; ++i)
    vecTmp.v[i] = pfData->v[i];

  res = AKFS_Rotate(pat, &vecTmp);

  for(i = 0; i < 3; ++i)
    pfData->v[i] = vecTmp.v[i];

  return res;

}
