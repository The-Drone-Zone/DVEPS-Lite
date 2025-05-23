## This is a Standalone script to download map tiles to the offline_tiles.db
## Once downloaded the tiles can be used in the GCS Offline mode.
## Modify "top_left_position" and "bottom_right_position" for new locations

import tkintermapview
import os
import sys
from Utils.Utils import get_root_dir


# Default is U of A Mall
def loadOfflineTiles(
    top_left=(32.2339879, -110.9566328), bottom_right=(32.2308002, -110.9502170)
):
    # to fix unicode printing error
    sys.stdout.reconfigure(encoding="utf-8")

    # This scripts creates a database with offline tiles.

    # specify the region to load (U of A Mall)
    top_left_position = top_left
    bottom_right_position = bottom_right
    zoom_min = 16
    zoom_max = 22

    # Get DB directory
    database_path = os.path.join(get_root_dir(), "offline_tiles.db")

    # create OfflineLoader instance
    loader = tkintermapview.OfflineLoader(path=database_path)
    # loader = tkintermapview.OfflineLoader(path=database_path,
    #                                       tile_server="https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}&s=Ga")

    # save the tiles to the database, an existing database will extended
    loader.save_offline_tiles(
        top_left_position, bottom_right_position, zoom_min, zoom_max
    )

    # You can call save_offline_tiles() multiple times and load multiple regions into the database.
    # You can also pass a tile_server argument to the OfflineLoader and specify the server to use.
    # This server needs to be set for the TkinterMapView when the database is used.
    # You can load tiles of multiple servers in the database. Which one then will be used depends on
    # which server is specified for the TkinterMapView.

    # print all regions that were loaded in the database
    loader.print_loaded_sections()


if __name__ == "__main__":
    loadOfflineTiles()
