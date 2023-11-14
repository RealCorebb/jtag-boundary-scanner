/*
 * JTAG Core library
 * Copyright (c) 2008 - 2023 Viveris Technologies
 *
 * JTAG Core library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * JTAG Core library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with JTAG Core library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/**
 * @file   drivers_list.h
 * @brief  drivers list struct
 * @author Jean-François DEL NERO <Jean-Francois.DELNERO@viveris.fr>
 */
typedef struct _drv_entry
{
	DRV_GETMODULEINFOS getinfosfunc;
	int sub_drv_id;
}drv_entry;

extern const drv_entry staticdrvs[];


