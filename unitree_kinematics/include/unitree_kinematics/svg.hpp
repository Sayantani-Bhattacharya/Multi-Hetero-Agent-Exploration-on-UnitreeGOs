// SVG.h
#ifndef SVG_H
#define SVG_H

#include <iostream>
#include <fstream>
#include <vector>

/// \file
/// \brief SVG File generator.

/// \brief SVG File generator class.
class SVG {
public:
    /// \brief Creates the .svg file 
    /// \param width width of the file
    /// \param height height of the file
    SVG(int width, int height);

    /// \brief Adds a rectangle to the .svg
    /// \param x x coordinate of the rectangle
    /// \param y y coordinate of the rectangle
    /// \param width width of the rectangle
    /// \param height height of the rectangle
    void addRectangle(int x, int y, int width, int height);

    /// \brief Creates a circle in the .svg
    /// \param cx x coordinate of the centre
    /// \param cy y coordinate of the centre
    /// \param r radius of the circle
    /// \param color color of the circle
    void addCircle(int cx, int cy, int r, const std::string& color);

    /// \brief Adds a line in the .svg
    /// \param x1 x of the start of the line
    /// \param y1 y of the start of the line
    /// \param x2 x of the end of the line
    /// \param y2 y of the end of the line
    /// \param stroke color of the line
    void addLine(int x1, int y1, int x2, int y2, const std::string& stroke);

    /// \brief Adds text to the .svg
    /// \param x x coordinate of the text box
    /// \param y y coordinate of the text box
    /// \param text the text that is written to the .svg
    void addText(int x, int y, const std::string& text);

    /// \brief saves the .svg to a specific file
    /// \param filename name of the file to save to
    /// \param additionalContent .the svg example that is modified
    void saveToFile(const std::string& filename, const std::string& additionalContent);
    
    /// \brief Gets the width of the image
    /// \return the width of the .svg
    int getWidth();
    
    /// \brief Gets the height of the image
    /// \return the height of the .svg
    int getHeight();

private:
    /// \brief width of the svg file
    int width;
    /// \brief height of the svg file
    int height;
    /// \brief detail elements of the svg file
    std::vector<std::string> elements;
};

#endif // SVG_H