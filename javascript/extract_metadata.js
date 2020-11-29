#!/usr/bin/env node
// taken from https://github.com/kws/GoProTelemetryExtract/blob/master/src/extract.js
const fs = require('fs');
const path = require('path');
const { extractInfo } = require('./helpers/mp4reader');

async function processAll(filenames, output_path) {
    for (let i = 0; i < filenames.length; i++) {
        const filename = filenames[i];
        const dir = path.dirname(filename);
        const ext = path.extname(filename);
        const basename = path.basename(filename, ext);
        const output = path.join(output_path, basename + '.json');

	if (!fs.existsSync(output)) {
	    await extractInfo(filename, output);
	} else {
	    console.log(`Skipping ${basename} as telemetry file already exists: ${output}`)
	}
    }
}

if (process.argv.length < 5) {
  console.error('Expected three arguments!');
  console.error('node extract_metadata_from_video.js BASEPATH_TO_VIDEO_FILE VIDEO_FILE OUTPUT_FOLDER');
  process.exit(1);
}

let basepath = process.argv[2];
let filename = process.argv[3];
let output_path = process.argv[4];

let video_path = basepath.concat('/').concat(filename)
let output_file = output_path.concat('/').concat(
  filename.concat('_telemetry.json'));
console.log("Input video path: ".concat(video_path));
console.log("Output file will be: ".concat(output_file));

//const filenames = process.argv.slice(2);
processAll([video_path], output_path)
    .then(() => console.log("Done"))
    .catch(err => console.error("An error occurred", err));

