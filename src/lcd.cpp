#include <Arduino.h>
#include "global.h"
#include <TFT_eSPI.h>


void TFT_printLine(String line, bool clearScreen = false)
{
    const int stringSize = 2;

    tft.setTextSize(stringSize);
    const int height = 16 / stringSize;
    const int width = 40 / stringSize;
    static String lines[height];
    static bool init = false;
    static int current_line = 0;
    // tft.fillScreen(TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setTextWrap(false);

    if (clearScreen)
    {
        init = false;
        current_line = -1;
    }

    if (!init)
    {
        for (size_t i = 0; i < height; i++)
        {

            lines[i].reserve(width);
            lines[i] = "";
        }
        init = true;
        tft.fillScreen(TFT_BLUE);
    }

        // line too long
    if (line.length() > width)
    {
        int position = 0;
        while (position < line.length())
        {
            TFT_printLine(line.substring(position, position + width));
            position += width;
        }

        return;
    }

    Serial.println(line);
    tft.setCursor(0, 0);

    // if we are starting with an empty screen we just need to tell it to print on the next line
    if (current_line < (height - 1))
    {
        current_line++;
    }
    else
    {
        // if we're at the bottom of the screen scroll lines up
        for (size_t i = 0; i < height - 1; i++)
        {
            lines[i] = lines[i + 1];
        }
    }

    // pad the string
    while (line.length() < width)
    {
        line += " ";
    }

    lines[current_line] = line;

    // lines[height - 1] = line;

    for (size_t i = 0; i < height; i++)
    {
        tft.println(lines[i]);
    }
}

void printScreenLine()
{
    TFT_printLine("", false);
}