#pragma once

//! @brief Colorize your shell. Be a full time retard  like us!
/*
                                       _------Q--\
                                     /~           )
                                   <_____________/
                                   /  _   )))))))))
              []                 /        (((((((((
            |~~~|               (____/'   ))))))))))
            |   |                  )))))))))))))))))           |\
            |   |                 ((((((((((((((((((          / |
            |   |     /~~\----------/|  //       \           | _/
            |   |<===|  ===]        ||//     \    \____     //'
          //|   |     \__/~~~~~~~~~~|^       _--~~~    ~~~-//
         // |   |                   |       /   ()    ()  // )
        //  |   |                   |      |  ()       _-//-~
       //   |   |                  ((((((((| ()       (_//
      //    |   |                   |  :   |            ~~---_
     //     |   |                   |  |    \  ()       ()    )
    //      |   |                   |  |     ~--__        __-~
   //       |___|                   |  |         |~//~~~~~
  //        //  \                 /  /          |//
 //        //     \             (___(___________|  
//      [==]       [==]
*/
namespace srrg2_core {

//! @brief full color macros. you can use them as follows:
//!        std::cout << COLOR_MACRO << __your_stuff__ << RESET;
#define RESET   "\033[0m"       /* Reset Shell Color */

#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */

#define BBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BRED     "\033[1m\033[31m"      /* Bold Red */
#define BGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BWHITE   "\033[1m\033[37m"      /* Bold White */

#define IBLACK   "\033[3m\033[30m"      /* Italic Black */
#define IRED     "\033[3m\033[31m"      /* Italic Red */
#define IGREEN   "\033[3m\033[32m"      /* Italic Green */
#define IYELLOW  "\033[3m\033[33m"      /* Italic Yellow */
#define IBLUE    "\033[3m\033[34m"      /* Italic Blue */
#define IMAGENTA "\033[3m\033[35m"      /* Italic Magenta */
#define ICYAN    "\033[3m\033[36m"      /* Italic Cyan */
#define IWHITE   "\033[3m\033[37m"      /* Italic White */

#define ULBLACK   "\033[4m\033[30m"      /* Underlined Black */
#define ULRED     "\033[4m\033[31m"      /* Underlined Red */
#define ULGREEN   "\033[4m\033[32m"      /* Underlined Green */
#define ULYELLOW  "\033[4m\033[33m"      /* Underlined Yellow */
#define ULBLUE    "\033[4m\033[34m"      /* Underlined Blue */
#define ULMAGENTA "\033[4m\033[35m"      /* Underlined Magenta */
#define ULCYAN    "\033[4m\033[36m"      /* Underlined Cyan */
#define ULWHITE   "\033[4m\033[37m"      /* Underlined White */

//ia unsupported
#define BLINKBLACK   "\033[6m\033[30m"      /* Fast Blinking Black */
#define BLINKRED     "\033[6m\033[31m"      /* Fast Blinking Red */
#define BLINKGREEN   "\033[6m\033[32m"      /* Fast Blinking Green */
#define BLINKYELLOW  "\033[6m\033[33m"      /* Fast Blinking Yellow */
#define BLINKBLUE    "\033[6m\033[36m"      /* Fast Blinking Blue */
#define BLINKMAGENTA "\033[6m\033[35m"      /* Fast Blinking Magenta */
#define BLINKCYAN    "\033[6m\033[36m"      /* Fast Blinking Cyan */
#define BLINKWHITE   "\033[6m\033[37m"      /* Fast Blinking White */

#define REVBLACK   "\033[7m\033[30m"      /* Reverse Black */
#define REVRED     "\033[7m\033[37m"      /* Reverse Red */
#define REVGREEN   "\033[7m\033[32m"      /* Reverse Green */
#define REVYELLOW  "\033[7m\033[33m"      /* Reverse Yellow */
#define REVBLUE    "\033[7m\033[34m"      /* Reverse Blue */
#define REVMAGENTA "\033[7m\033[35m"      /* Reverse Magenta */
#define REVCYAN    "\033[7m\033[36m"      /* Reverse Cyan */
#define REVWHITE   "\033[7m\033[37m"      /* Reverse White */


//! ------------------------------------------------------------------------ !//
//! ------------------------------------------------------------------------ !//
//! @brief less bothering color macros, for base colors. usage
//!        std::cout << C_COLOR(__your_stuff__) << std::endl;
#define FG_BLACK(s)     "\033[30m" << s << "\033[0m"
#define FG_RED(s)       "\033[31m" << s << "\033[0m"
#define FG_GREEN(s)     "\033[32m" << s << "\033[0m"
#define FG_YELLOW(s)    "\033[33m" << s << "\033[0m"
#define FG_BLUE(s)      "\033[34m" << s << "\033[0m"
#define FG_MAGENTA(s)   "\033[35m" << s << "\033[0m"
#define FG_CYAN(s)      "\033[36m" << s << "\033[0m"
#define FG_WHITE(s)     "\033[37m" << s << "\033[0m"


//! @brief less bothering color macros, for bold colors. usage
//!        std::cout << C_COLOR(__your_stuff__) << std::endl;
#define FG_BBLACK(s)     "\033[1m\033[30m" << s << "\033[0m"
#define FG_BRED(s)       "\033[1m\033[31m" << s << "\033[0m"
#define FG_BGREEN(s)     "\033[1m\033[32m" << s << "\033[0m"
#define FG_BYELLOW(s)    "\033[1m\033[33m" << s << "\033[0m"
#define FG_BBLUE(s)      "\033[1m\033[34m" << s << "\033[0m"
#define FG_BMAGENTA(s)   "\033[1m\033[35m" << s << "\033[0m"
#define FG_BCYAN(s)      "\033[1m\033[36m" << s << "\033[0m"
#define FG_BWHITE(s)     "\033[1m\033[37m" << s << "\033[0m"


//! @brief less bothering color macros, for underlined colors. usage
//!        std::cout << C_COLOR(__your_stuff__) << std::endl;
#define FG_ULBLACK(s)     "\033[4m\033[30m" << s << "\033[0m"
#define FG_ULRED(s)       "\033[4m\033[31m" << s << "\033[0m"
#define FG_ULGREEN(s)     "\033[4m\033[32m" << s << "\033[0m"
#define FG_ULYELLOW(s)    "\033[4m\033[33m" << s << "\033[0m"
#define FG_ULBLUE(s)      "\033[4m\033[34m" << s << "\033[0m"
#define FG_ULMAGENTA(s)   "\033[4m\033[35m" << s << "\033[0m"
#define FG_ULCYAN(s)      "\033[4m\033[36m" << s << "\033[0m"
#define FG_ULWHITE(s)     "\033[4m\033[37m" << s << "\033[0m"


//! @brief less bothering color macros, for italic colors. usage
//!        std::cout << C_COLOR(__your_stuff__) << std::endl;
#define FG_IBLACK(s)     "\033[3m\033[30m" << s << "\033[0m"
#define FG_IRED(s)       "\033[3m\033[31m" << s << "\033[0m"
#define FG_IGREEN(s)     "\033[3m\033[32m" << s << "\033[0m"
#define FG_IYELLOW(s)    "\033[3m\033[33m" << s << "\033[0m"
#define FG_IBLUE(s)      "\033[3m\033[34m" << s << "\033[0m"
#define FG_IMAGENTA(s)   "\033[3m\033[35m" << s << "\033[0m"
#define FG_ICYAN(s)      "\033[3m\033[36m" << s << "\033[0m"
#define FG_IWHITE(s)     "\033[3m\033[37m" << s << "\033[0m"


//! @brief less bothering color macros, for reversed colors. usage
//!        std::cout << C_COLOR(__your_stuff__) << std::endl;
#define BG_BLACK(s)     "\033[7m\033[30m" << s << "\033[0m"
#define BG_RED(s)       "\033[7m\033[31m" << s << "\033[0m"
#define BG_GREEN(s)     "\033[7m\033[32m" << s << "\033[0m"
#define BG_YELLOW(s)    "\033[7m\033[33m" << s << "\033[0m"
#define BG_BLUE(s)      "\033[7m\033[34m" << s << "\033[0m"
#define BG_MAGENTA(s)   "\033[7m\033[35m" << s << "\033[0m"
#define BG_CYAN(s)      "\033[7m\033[36m" << s << "\033[0m"
#define BG_WHITE(s)     "\033[7m\033[37m" << s << "\033[0m"


}
