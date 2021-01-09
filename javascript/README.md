# GoProMetadataExtractor
Simple project that extract GoPro telemetry data to json

## Installation

``` bash
npm install
```

## Extract metadata
``` bash
node extract_metadata.js $BASEPATH_TO_VIDEO_FILE $VIDEO_FILE $OUTPUT_FOLDER
e.g.: 
node extract_metadata.js /home/myhome/path GH015744.MP4 /home/myhome/output_json_path
```

## infos
https://gopro.github.io/gpmf-parser/