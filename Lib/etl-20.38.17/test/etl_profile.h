///\file

/******************************************************************************
The MIT License(MIT)

Embedded Template Library.
https://github.com/ETLCPP/etl
https://www.etlcpp.com

Copyright(c) 2017 John Wellbelove

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#ifndef ETL_PROFILE_H_INCLUDED
#define ETL_PROFILE_H_INCLUDED

#define ETL_THROW_EXCEPTIONS
#define ETL_VERBOSE_ERRORS
#define ETL_CHECK_PUSH_POP
#define ETL_ISTRING_REPAIR_ENABLE
#define ETL_IVECTOR_REPAIR_ENABLE
#define ETL_IDEQUE_REPAIR_ENABLE
#define ETL_ICIRCULAR_BUFFER_REPAIR_ENABLE
#define ETL_IN_UNIT_TEST
//#define ETL_DEBUG_COUNT
#define ETL_ARRAY_VIEW_IS_MUTABLE

#define ETL_MESSAGE_TIMER_USE_ATOMIC_LOCK
#define ETL_CALLBACK_TIMER_USE_ATOMIC_LOCK

#define ETL_POLYMORPHIC_RANDOM

#define ETL_POLYMORPHIC_BITSET
#define ETL_POLYMORPHIC_DEQUE
#define ETL_POLYMORPHIC_FLAT_MAP
#define ETL_POLYMORPHIC_FLAT_MULTIMAP
#define ETL_POLYMORPHIC_FLAT_SET
#define ETL_POLYMORPHIC_FLAT_MULTISET
#define ETL_POLYMORPHIC_FORWARD_LIST
#define ETL_POLYMORPHIC_LIST
#define ETL_POLYMORPHIC_MAP
#define ETL_POLYMORPHIC_MULTIMAP
#define ETL_POLYMORPHIC_SET
#define ETL_POLYMORPHIC_MULTISET
#define ETL_POLYMORPHIC_QUEUE
#define ETL_POLYMORPHIC_STACK
#define ETL_POLYMORPHIC_REFERENCE_FLAT_MAP
#define ETL_POLYMORPHIC_REFERENCE_FLAT_MULTIMAP
#define ETL_POLYMORPHIC_REFERENCE_FLAT_SET
#define ETL_POLYMORPHIC_REFERENCE_FLAT_MULTISET
#define ETL_POLYMORPHIC_UNORDERED_MAP
#define ETL_POLYMORPHIC_UNORDERED_MULTIMAP
#define ETL_POLYMORPHIC_UNORDERED_SET
#define ETL_POLYMORPHIC_UNORDERED_MULTISET
#define ETL_POLYMORPHIC_STRINGS
#define ETL_POLYMORPHIC_POOL
#define ETL_POLYMORPHIC_VECTOR
#define ETL_POLYMORPHIC_INDIRECT_VECTOR

#if defined(ETL_FORCE_TEST_CPP03_IMPLEMENTATION)
  #define ETL_FUNCTION_FORCE_CPP03_IMPLEMENTATION
  #define ETL_PRIORITY_QUEUE_FORCE_CPP03_IMPLEMENTATION
  #define ETL_QUEUE_ATOMIC_FORCE_CPP03_IMPLEMENTATION
  #define ETL_VARIANT_FORCE_CPP03_IMPLEMENTATION
  #define ETL_VECTOR_FORCE_CPP03_IMPLEMENTATION
  #define ETL_QUEUE_FORCE_CPP03_IMPLEMENTATION
  #define ETL_QUEUE_MPMC_MUTEX_FORCE_CPP03_IMPLEMENTATION
  #define ETL_QUEUE_ISR_FORCE_CPP03_IMPLEMENTATION
  #define ETL_QUEUE_LOCKED_FORCE_CPP03_IMPLEMENTATION
  #define ETL_OPTIONAL_FORCE_CPP03_IMPLEMENTATION
  #define ETL_LARGEST_TYPE_FORCE_CPP03_IMPLEMENTATION
  #define ETL_TYPE_SELECT_FORCE_CPP03_IMPLEMENTATION
  #define ETL_UNINITIALIZED_BUFFER_FORCE_CPP03_IMPLEMENTATION
  #define ETL_CRC_FORCE_CPP03_IMPLEMENTATION
  #define ETL_MEM_CAST_FORCE_CPP03_IMPLEMENTATION
  #define ETL_OBSERVER_FORCE_CPP03_IMPLEMENTATION
  #define ETL_MESSAGE_PACKET_FORCE_CPP03_IMPLEMENTATION
  #define ETL_OBSERVER_FORCE_CPP03_IMPLEMENTATION
  #define ETL_MESSAGE_ROUTER_FORCE_CPP03_IMPLEMENTATION
  #define ETL_FSM_FORCE_CPP03_IMPLEMENTATION
  #define ETL_DELEGATE_FORCE_CPP03_IMPLEMENTATION
  #define ETL_SINGLETON_FORCE_CPP03_IMPLEMENTATION
  #define ETL_BYTE_FORCE_CPP03_IMPLEMENTATION
  #define ETL_LIST_FORCE_CPP03_IMPLEMENTATION
  #define ETL_FORWARD_LIST_FORCE_CPP03_IMPLEMENTATION
  #define ETL_FLAT_SET_FORCE_CPP03_IMPLEMENTATION
  #define ETL_FLAT_MULTISET_FORCE_CPP03_IMPLEMENTATION
  #define ETL_VARIANT_POOL_FORCE_CPP03_IMPLEMENTATION
#endif

#if defined(ETL_FORCE_TEST_CPP11)
  #define ETL_OVERLOAD_FORCE_CPP11
  #define ETL_VARIANT_FORCE_CPP11
#endif

#include "../include/etl/profiles/determine_compiler_language_support.h"
#include "../include/etl/profiles/determine_compiler_version.h"
#include "../include/etl/profiles/determine_development_os.h"

//#if ETL_CPP17_NOT_SUPPORTED
//  #error THE UNIT TESTS REQUIRE C++17 SUPPORT
//#endif

#if defined(ETL_COMPILER_GCC)
  #if (ETL_COMPILER_VERSION < 8)
    #define ETL_TEMPLATE_DEDUCTION_GUIDE_TESTS_DISABLED
  #endif
#endif

#if defined(ETL_DEVELOPMENT_OS_WINDOWS)
  #define ETL_TARGET_OS_WINDOWS
#elif defined(ETL_DEVELOPMENT_OS_LINUX)
  #define ETL_TARGET_OS_LINUX
#else
  #define ETL_TARGET_OS_GENERIC
#endif

#if !((ETL_CPP20_SUPPORTED && !defined(ETL_NO_STL)) || defined(__BYTE_ORDER__))
  #define ETL_ENDIAN_NATIVE 0
#endif

#endif