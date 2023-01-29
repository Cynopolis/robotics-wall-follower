#include "MemorySize.h"
class SerialMessage{
    public:
        /**
         * @brief Construct a new Serial Message object
         * @param baud_rate the baud rate of the serial connection
         */
        SerialMessage(int baud_rate);

        /**
         * @brief Update the SerialMessage object and parse any data that's available
         */
        void update();

        /**
         * @brief Print a message to the serial monitor with a newline
         * @param message the message to print
         */
        void println(char message[]);

        /**
         * @brief Print a message to the serial monitor
         * @param message the message to print
         */
        void print(char message[]);

        /**
         * @brief Returns true if there is new data available
         * @return true if there is new data available
         */
        bool isNewData();

        /**
         * @brief Clears the new data flag
         */
        void clearNewData();

        /**
         * @brief Return a pointer to the args array
         * @return a pointer to the args array
         */
        int * getArgs();
        int getArgsLength();
    private:
        void readSerial();
        void parseData();

        bool new_data = false;
        char data[num_chars]; // an array to store the received data
        char temp_data[num_chars]; // an array that will be used with strtok()
        const static int args_length = 30;
        int args[args_length];
        const char startMarker = '!';
        const char endMarker = ';';
};