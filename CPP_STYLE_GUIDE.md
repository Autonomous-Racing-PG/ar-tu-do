<!-- MarkdownTOC -->

- [Terminology](#terminology)
- [Autoformatting](#autoformatting)
- [Naming](#naming)
    - [Packages](#packages)
    - [Topics / Services](#topics--services)
    - [Files](#files)
        - [Classes](#classes)
        - [Function](#function)
    - [Variables](#variables)
        - [Member variables](#member-variables)
        - [Constants](#constants)
            - [ROS Topics / Services](#ros-topics--services)
            - [ROS Parameters](#ros-parameters)
        - [Global variables](#global-variables)
    - [Namespaces](#namespaces)
- [License statements](#license-statements)
- [Formatting](#formatting)
    - [Line length](#line-length)
- [Include guard](#include-guard)
- [Documentation](#documentation)
- [Comments](#comments)
    - [Function summary](#function-summary)
- [Text output](#text-output)
- [Macros](#macros)
- [Preprocessor directives](#preprocessor-directives)
- [Output arguments](#output-arguments)
- [Namespaces](#namespaces-1)
- [Inheritance](#inheritance)
    - [Multiple inheritance](#multiple-inheritance)
- [Exceptions](#exceptions)
- [Enumerations](#enumerations)
- [Globals](#globals)
- [Static class variables](#static-class-variables)
- [Magic Numbers](#magic-numbers)
- [Assertions](#assertions)
- [Deprecation](#deprecation)
- [Main function](#main-function)
- [Zero cost abstraction](#zero-cost-abstraction)

<!-- /MarkdownTOC -->

This document is loosely based on the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide) and defines a style guide to be followed in writing C++ code in the Autonomous-Racing-PG. This guide SHOULD be followed as close as possible but MUST NOT be seen as holy book that has to be followed without any question. 

In case this document does not describe something, try to finde a reasonable solution. In case of discussion: [Be excellent to each other](https://www.youtube.com/watch?v=rph_1DODXDU).

<a id="terminology"></a>
# Terminology

This document uses the following shortcuts:

- **CamelCased**: A name is *CamelCased* if it starts with an uppercase letter, each words starts with an uppercase letter and no special character (e.g. underscore) are between the words e.g. MyNameIsTed.
- **camelCased**: A name is *camelCased* if it starts with a lowercase letter, each words starts with an uppercase letter and no special character (e.g. underscore) are between the words e.g. myNameIsTed.
- **snake_cased**: A name is *snake_cased* if all letters are in lowercase and an underscore is used as a special character between each word e.g. my_name_is_ted.
- **SNAKE_CASED**: A name is  *SNAKE_CASED* if all letters are in uppercase and an underscore is used as a special character between each word MY_NAME_IS_TED.

The words **SHOULD**, **SHOULD NOT**, **MUST**, **MUST NOT** and **MAY** are used with the semantic described by [RFC 2119](https://www.ietf.org/rfc/rfc2119.txt).

<a id="autoformatting"></a>
# Autoformatting 

It is heavily recommended to use the `clang-format` tool with the provided `.clang-format`.
See the [wiki](https://github.com/Autonomous-Racing-PG/ros.package/wiki/Formatting-Cpp-and-Python-code) for more information.

<a id="naming"></a>
# Naming

Names SHOULD make it easy for readers to understand your code.

Readers SHOULD need to make as few assumptions and guesses about your code as possible.

Avoid abbreviations and acronyms.
Acronyms MAY be used if they can be understood without domain knowledge.
Abbreviations MAY be used if their scope is very small.

<a id="packages"></a>
## Packages
A ROS package name SHOULD be *snaked_cased*.

<a id="topics--services"></a>
## Topics / Services
A ROS topic and service name SHOULD be *snaked_cased*.

<a id="files"></a>
## Files
All filenames SHOULD be *snake_cased*.

All C++ source files SHOULD have the extension `.cpp`.

All C++ header files SHOULD have the extension `.h` and SHOULD be placed in the `include` directory of the ROS Packages `src` directory.

If a file mainly contains definition or implementation of a class, the file SHOULD be named *snake_cased* after the class name e.g. `class MyOwnClass` results to the filename `my_own_class.h/.cpp` 

A filename SHOULD be descriptive.

You MAY split the implementation on a by-function basis e.g.
> include/my/example.h

```C++
#pragma once
namespace my {
    class Example {
        public:
        int myFunction();
        int doStuff();
    };
}
```
> src/my/example_my_function.cpp

```C++
#pragma once
namespace my {
    int Example::myFunction() {
        return 0;
    }
}
```
> src/my/example_do_stuff.cpp

```C++
#pragma once
namespace my {
    int Example::doStuff() {
        return 1;
    }
}
```

<a id="classes"></a>
### Classes

A class name SHOULD be *CamelCased* e.g. `class MyVector`.
Short acronyms MAY be in all capitals e.g. `MyMPC` (see [MPC](https://en.wikipedia.org/wiki/Model_predictive_control)) or `MyROSNode`.

A class name SHOULD NOT be composed of more then three words.

<a id="function"></a>
### Function

A function name SHOULD be *camelCased*. Arguments SHOULD be *snake_cased*.
E.g.
```C++
int8_t myFunction(std::vector<int8_t>& my_argument, const char* my_argument_2 = nullptr);
```

As functions usually do something, their name SHOULD describe clearly what they do.

<a id="variables"></a>
## Variables

A variable name SHOULD be *snake_cased* and SHOULD NOT be cryptic, but try not to make them unnecessary long.

Counter variables MAY only use a single character variable like `i`, `j` or `k`.
Iterator variable SHOULD have `it` in its name.
E.g.
```C++
std::vector<std::array<uint8_t, 2>>::iterator position_it;
```
```C++
for (size_t i = 0; i < 100; i++)
{
    ROS_DEBUG("%d", i);
}
```

<a id="member-variables"></a>
### Member variables

All member variable names SHOULD be *snake_cased* with `m_` as prefix.
E.g.
```C++
class MyClass
{
    private:
    int64_t m_element_counter;
    MyClass* m_parent;
};
```

The prefix MAY be omitted if a member variable is `public`.
E.g.
```C++
struct MyClass
{
    int64_t element_counter;
    MyClass* parent;
};
```
```C++
class MyClass
{
    public:
    int64_t element_counter;
    MyClass* parent;
};
```

<a id="constants"></a>
### Constants

All Constants names SHOULD be *SNAKE_CASED*.

Try to use `constexpr`, `enum` and `enum class` before resorting to `#define` or `const`.
E.g.
```C++
#define MY_MACRO_CONST 10                   // Bad as this macro may be already defined somewhere else
const unsigned MY_CONST_CONST = 10;         // Better as one would get a compile time error if it was defined somewhere else!
constexpr unsigned MY_CONSTEXPR_CONST = 10; // Good as the compiler tries to initialize the variable on compile time if possible.
```

See [Using constexpr to Improve Security, Performance and Encapsulation in C++](https://smartbear.de/blog/develop/using-constexpr-to-improve-security-performance-an/?l=ua)
for more information on why `constexpr` is awesome.

<a id="ros-topics--services"></a>
#### ROS Topics / Services

If a ROS topics or services name is stored in a constant, the constant's name SHOULD beginn with *TOPIC_*.

<a id="ros-parameters"></a>
#### ROS Parameters

If a ROS parameters name is stored in a constant, the constant's name SHOULD beginn with *PARAMETER_*.

<a id="global-variables"></a>
### Global variables

Global variables SHOULD NOT be used. See [this](#globals).

Global variables names SHOULD be *snake_cased* with `g_`as prefix.
E.g.
```C++
// Reason why this global variable is really necessary
my::own::Database g_database;
```

<a id="namespaces"></a>
## Namespaces

A Namespace MUST be *snake_cased*.

Try to avoid long names.

<a id="license-statements"></a>
# License statements
Each source and header file MAY contain a license and copyright statement.

<a id="formatting"></a>
# Formatting

Any `C++` code SHOULD be formatted using the provided `.clang-format` file.

In rare cases where formatting is not wished for, you SHOULD use `// clang-format off`
to disable clang-format.
E.g.
```C++
// clang-format off
int g_magic_numbers[] = {
    1,2,3,
    4,5,6,
    7,8,9,
      0
};
// clang-format on
```
See [disabling-formatting-on-a-piece-of-code](https://clang.llvm.org/docs/ClangFormatStyleOptions.html#disabling-formatting-on-a-piece-of-code) for more information.

<a id="line-length"></a>
## Line length

A line of code SHOULD NOT have more then 120 characters.

<a id="include-guard"></a>
# Include guard

You SHOULD NOT use `#ifndef` based include guards, as they are cumbersome and prone for errors.
E.g.
```C++
// BAD
#ifndef MY_HEADER_FILE_H
#define MY_HEADER_FILE_H
/* CODE */
#endif
```
Instead you SHOULD use `#pragma once`.
E.g.
```C++
// GOOD
#pragma once
/* CODE */
```

<a id="documentation"></a>
# Documentation

Documentation explains the high-level structure of the code.
It also provides information on how to use the project to those who didn't read the code.

Code SHOULD be documented in a [doxygen](http://www.doxygen.nl/manual/docblocks.html) compatible fashion.

Function and class summaries SHOULD be omitted if they do not provide more information than the name:

```C++
// BAD
/**
 *  Returns the size of the list
 */
int List::getSize()
{
    return this->m_size;
}
```

Conversely, classes and functions SHOULD be named so that it is obvious what they do:

```C++
// BAD
/**
 *   Publishes the Ackermann driving parameters
 *   @param a: The steering angle
 *   @param s: The speed
 */
void send(double a, double s)
```

```C++
// GOOD
void publishAckermannDrivingParameters(double steering_angle, double speed)
```

<a id="comments"></a>
# Comments

Comments SHOULD explain why something is done, not what is being done.
They SHOULD explain counter-intuitive semantics, assumptions, invariants and workarounds.
However, code that needs explanatory comments can often be avoided by good naming.

```C++
// BAD
// Increase height by 0.01
height += 0.01;
```

```C++
// BETTER
// Adjust height to prevent self-collisions
height += 0.01;
```

```C++
// GOOD
height += NO_SELF_COLLISION_MARGIN;
```

```C++
// BAD
// Vector to store length information
std::vector<int8_t> vec;

// ...

// Subtract 1 of each element of the int8_t vector *vec*
for (auto& element : vec) {
    element -= 1;
}
```

```C++
// GOOD
std::vector<int8_t> lengths;

// ...

/**
 *   As the vector *lengths* contains length information
 *   and the API for the external lib *my_crappy_lib*
 *   expects not a vector of length information but
 *   a vector of the last valid index, we have to 
 *   offset the elements in the vector by -1
 */
for (auto& item_length : lengths) {
    item_length -= 1;
}
```

Comments SHOULD NOT be used to structure the code.
Instead of separating a long function with comments, it SHOULD be split into multiple shorter functions.

<a id="function-summary"></a>
## Function summary

You SHOULD only add a summary to the declaration of a function.

<a id="text-output"></a>
# Text output

You SHOULD NOT use `printf` or `std::cout`, instead use [rosconsole](http://wiki.ros.org/rosconsole) as it supports:

- Colored output
- Verbosity levels
- Automatically publishing the output to the ROS topic `/rosout`
- logging to disk

<a id="macros"></a>
# Macros

You SHOULD NOT use macros. Use `constexpr` or `inline` functions if possible, as macros are neither typed nor scoped and prone for errors.
E.g.
```C++
// BAD
#define SQUARE(x) x*x
#define DO_WORK(x) x -= 5;
// NOT BAD (but also not GOOD)
#define SQUARE(x) ((x)*(x))
#define DO_WORK(x) do{x -= 5;}while(false);
```
```C++
// GOOD
template<typename T>
inline constexpr T SQUARE(T x)
{
    return x * x;
}
template<typename T>
inline void DO_WORK(T& x)
{
    x -= 5;
}
```

<a id="preprocessor-directives"></a>
# Preprocessor directives

You SHOULD NOT use preprocessor directives, as they are prone for errors. Use `template` and `inline` function as subtitution if possible.

<a id="output-arguments"></a>
# Output arguments

You SHOULD NOT use output arguments, as they obfuscate side effects.

<a id="namespaces-1"></a>
# Namespaces

Usage of namespaces is highly encouraged.

The source files MAY be organized by namespace e.g. the file `my_class.cpp` with the implementation of `my::own::MyClass` MAY be put in the directory `src/my/own/`.

The usage of nested namespace (e.g. ```namespace A::B::C::D::E::F::G::H::J {}```) is encouraged.

You SHOULD NOT use a using-directive in a header file.
E.g.
```C++
// BAD
#include <vector>
namespace A
{
    using namespace std;
    vector<int> myFunction();
}
```

<a id="inheritance"></a>
# Inheritance

You MUST declare an overridden virtual function with the identifiers `virtual` AND `override` to clarify whether or not a given function is virtual (and overridden).
```C++
// BAD
class Base {
    public:
    virtual foo();
    virtual ~Base();
};
class A : public Base {
    public:
    foo() {
        // Do Stuff
    }
    ~A() = default;
};
```
```C++
// GOOD
class Base {
    public:
    virtual foo() = 0;
    virtual ~Base() = 0;
};
class A : public Base {
    public:
    virtual foo() override {
        // Do Stuff
    }
    virtual ~A() override = default;
};
```
See [override](https://en.cppreference.com/w/cpp/language/override) and [virtual](https://en.cppreference.com/w/cpp/language/virtual) for more information.

If a class is used to define a common interface for several possible implementations, virtual member functions SHOULD be used,
as type casting otherwise could lead to hard to debug errors.

It is encouraged to use pure virtual classes as a common interface.

It is encouraged to use `virtual` only with moderation.

A `virtual` class SHOULD have a `virtual` destructor.

<a id="multiple-inheritance"></a>
## Multiple inheritance

It is encouraged to use multiple inheritance only with moderation. Try to avoid it if possible.

<a id="exceptions"></a>
# Exceptions

Exceptions SHOULD be preferred over the usage of error codes. If you are using error codes, it is highly encouraged to use an `enum` or `enum class` as return type.

You SHOULD document what kind of exception a given function might throw.
E.g.
```C++
/**
 * @throws IOException Description why and when an IOException is thrown
 * @throws MyException Description why and when a MyException is thrown
 */
void myFunction() {
    /* DO STUFF */
}
```

You MUST NOT throw an exception from a destructor.

You MUST NOT throw an exception from a callback function.

If your code can be interrupted by an exception, you MUST make sure such an exception does not lead to an undefined or otherwise broken state e.g. forgetting to free a `mutex`. This MAY be accomplished by things like a [lock_guard](https://en.cppreference.com/w/cpp/thread/lock_guard).

<a id="enumerations"></a>
# Enumerations

To prevent conflicts between enums, they SHOULD be either namespaced, classed or declared as a `enum class`.
E.g.
```C++
namespace namespaced_enum {
    enum OptCode {
        A,
        B,
        C
    };
}
class classed_enum {
    public:
    enum OptCode {
        A,
        B,
        C
    };
}
enum class OptCode {
    A,
    B,
    C
};
```

<a id="globals"></a>
# Globals

You SHOULD NOT use global variables and functions, as they create a hidden (global) state and are prone for threading and link time errors. ([some more reasons] (http://wiki.c2.com/?GlobalVariablesAreBad))

<a id="static-class-variables"></a>
# Static class variables

You SHOULD NOT use `static` class member variables, as they create a hidden (global) state and are prone for threading errors.

<a id="magic-numbers"></a>
# Magic Numbers

You SHOULD NOT use magic numbers in the source code.
E.g.
```C++
// BAD
int main() {
    auto response_code = doSomething();
    if (response_code == 400) {
        return 1;
    }
    return 0;
}
```
```C++
// GOOD
int main() {
    auto response_code = doSomething();
    if (response_code == HTTP_STATUS_CODE::BadRequest) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
```

If a number has a special meaning, usage of an `constexpr`, `enum`/`enum class` to address it is strongly recommended.

You SHOULD NOT replace a magic number with a magic constant.
E.g.
```C++
// BAD
int main() {
    auto response_code = doSomething();
    if (response_code == OTHER_NUMBERS::FOUR_HUNDRED) {
        return NUMBER::ONE;
    }
    return NUMBER::ZERO;
}
```
<a id="assertions"></a>
# Assertions

The usage of assertions to check invariants and assumptions is highly encouraged.

You SHOULD use the ROS assertions (`ROS_COMPILE_ASSERT(cond)`,`ROS_STATIC_ASSERT(cond)`,`ROS_ASSERT(cond)`,`ROS_ASSERT_MSG(cond, "format string", ...)`,`ROS_ASSERT_CMD(cond, function())`) provided in `ros/assert.h`.
E.g.
```C++
int64_t divide(int64_t a, int64_t b) {    
    ROS_ASSERT(b != 0);
    ROS_ASSERT_MSG(b != 0, "Division by zero");
    ROS_ASSERT_CMD(b != 0, callRufus());
    return a / b;
}
```
See [http://docs.ros.org](http://docs.ros.org/electric/api/rosconsole/html/assert_8h.html) for more information.

You SHOULD NOT do work in an assertion.
E.g.
```C++
// BAD
int64_t sum(int64_t a, int64_t b) {
    ROS_ASSERT_MSG((a + b) = 0, "If you divide the number %d by %d you get %d", a, b, divide(a,b));
    return a + b;
}
```
<a id="deprecation"></a>
# Deprecation

You SHOULD use the [deprecated attribute](https://en.cppreference.com/w/cpp/language/attributes/deprecated) to declare
a `struct`, `class`, `enum`, `function` or other elements as deprecated.
E.g.
```C++
namespace [[deprecated]] my_namespace {
    class MyClass;
}
```

You SHOULD use `#pragma GCC warning "MSG"` or `#pragma GCC error "MSG"` do deprecate whole files.
E.g.
```C++
#pragma GCC warning "Files was moved to another location"
#include <new/location/filename.h>
```
or
```C++
#pragma GCC error "Class MyClass was removed in Version 2.0"
// Here was once a class called MyClass
```

<a id="main-function"></a>
# Main function

The `main` function of a program SHOULD be in a seperate `.c`/`.cpp` file.

<a id="zero-cost-abstraction"></a>
# Zero cost abstraction

Zero cost abstractions like `std::array` or `std::lock_guard` SHOULD always be prefered.
```C++
// BAD
int64_t arr[3] = {1,2,3};
// BETTER
auto arr = std::array<int64_t, 3>({1,2,3});
// OR
std::array<int64_t, 3> arr = {1,2,3};
// GOOD (but experimental)
auto arr = std::experimental::make_array(1,2,3);
// See: https://en.cppreference.com/w/cpp/experimental/make_array

//BAD
void foo() {
    resource_mutex.lock();
    // DO STUFF
    resource_mutex.unlock();
}
// GOOD
void foo() {
    {
        auto lock_guard = std::lock_guard<std::mutex>(resource_mutex)
        // DO STUFF
    }
}
```
