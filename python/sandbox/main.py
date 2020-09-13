import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import pynfinitam


def print_mat(m: pynfinitam.Matrix4f):
    for y in range(4):
        for x in range(4):
            print(m.at(x, y), end=' ')
        print()


def main():
    settings = pynfinitam.Settings()

    sequence_dir = "C:/spaint/build/bin/apps/spaintgui/sequences/Teddy"
    calib_filename = os.path.join(sequence_dir, "calib.txt")
    calib = pynfinitam.ITMRGBDCalib()
    pynfinitam.read_rgbd_calib(calib_filename, calib)

    path_generator = pynfinitam.ImageMaskPathGenerator(
        os.path.join(sequence_dir, "frame-%06i.color.png"),
        os.path.join(sequence_dir, "frame-%06i.depth.png")
    )

    initial_frame_no = 0
    image_file_reader: pynfinitam.ImageSourceEngine = pynfinitam.ImageMaskFileReader(
        os.path.join(sequence_dir, "calib.txt"), path_generator, initial_frame_no
    )
    rgb_image = pynfinitam.ORUChar4Image(image_file_reader.get_rgb_image_size(), True, True, False)
    raw_depth_image = pynfinitam.ORShortImage(image_file_reader.get_depth_image_size(), True, True, False)

    basic_engine = pynfinitam.ITMBasicEngine(
        settings, calib, pynfinitam.Vector2i(640, 480), pynfinitam.Vector2i(640, 480)
    )

    img_size = pynfinitam.Vector2i(640, 480)
    out_rgb_image = pynfinitam.ORUChar4Image(img_size, True, True, False)
    out_depth_image = pynfinitam.ORFloatImage(img_size, True, True, False)

    # depth_visualiser = pynfinitam.DepthVisualiserFactory.make_depth_visualiser(settings.device_type)
    # visualisation_engine = pynfinitam.ITMVisualisationEngineFactory.make_visualisation_engine(settings.device_type)
    # render_state = pynfinitam.ITMRenderStateFactory.create_render_state(img_size, settings.scene_params, settings.get_memory_type())

    frame_idx = 0
    while image_file_reader.has_more_images():
        print(frame_idx)
        image_file_reader.get_images(rgb_image, raw_depth_image)
        basic_engine.process_frame(rgb_image, raw_depth_image)

        basic_engine.get_image(out_rgb_image, pynfinitam.IMAGE_COLOUR_FROM_VOLUME, basic_engine.get_tracking_state().pose_d, calib.intrinsics_d)
        cv2.imshow("Out", np.array(out_rgb_image, copy=False))
        cv2.waitKey(1)

        # pynfinitam.DepthVisualisationUtil.generate_depth_from_voxels(
        #     out_depth_image, basic_engine.get_scene(), basic_engine.get_tracking_state().pose_d, calib.intrinsics_d,
        #     render_state, pynfinitam.DT_ORTHOGRAPHIC, visualisation_engine, depth_visualiser, settings
        # )
        # plt.imshow(out_depth_image)
        # plt.draw()
        # plt.pause(0.01)

        frame_idx += 1

    cv2.waitKey()


if __name__ == "__main__":
    main()
