##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = t_mobile

FREERTOS = ../FreeRTOS/Source

DEFS += -I$(FREERTOS)/include -I$(FREERTOS)/portable/ARM_CM3 -I.

OBJS = mytasks.o jfes.o periph.o

OBJS += $(FREERTOS)/croutine.o $(FREERTOS)/event_groups.o
OBJS += $(FREERTOS)/queue.o $(FREERTOS)/tasks.o $(FREERTOS)/list.o
OBJS += $(FREERTOS)/portable/MemMang/heap_4.o
OBJS += $(FREERTOS)/portable/ARM_CM3/port.o

LDSCRIPTPR = f1/stm32f103x8.ld

include Makefile.include

