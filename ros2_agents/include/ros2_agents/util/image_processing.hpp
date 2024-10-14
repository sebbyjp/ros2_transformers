#include <vector>
#include <array>
#include <cstdint>

#include <sensor_msgs/msg/image.hpp>

namespace mbodied
{
    
using sensor_msgs::msg::Image;

std::vector<std::vector<std::array<uint8_t, 3> > >image_to_array(Image *msg)
{
    // convert msg from Image to numpy array
    std::vector<std::vector<std::array<uint8_t, 3> > > image;

    for (int i = 0; i < static_cast<int>(msg->height); i++)
    {
        image.push_back(std::vector<std::array<uint8_t, 3> >(msg->width));

        for (int j = 0; j < static_cast<int>(msg->width); j++)
        {
            image[i][j] = std::array<uint8_t, 3>();

            for (int k = 0; k < 3; k++)
            {
                image[i][j][k] = msg->data[i * msg->width * 3 + j * 3 + k];
            }
        }
    }
    return image;
}

} // namespace mbodied