

const fs = require('fs');
const path = require('path');
const MP4Box = require('mp4box');
const goproTelemetry = require('gopro-telemetry');

/**
 * Streams any size file.
 * @param filename
 */
function streamData(filename) {
    return new Promise((resolve, reject) => {
        const mp4boxfile = MP4Box.createFile();
        mp4boxfile.onError = function(e) {
            reject(e);
        };
        mp4boxfile.onReady = function(videoData) {
            resolve({ videoData, mp4boxfile});
        };

	const chunkSize = 100*1024*1024; // 100Mb
	const stream = fs.createReadStream(filename, {'highWaterMark': chunkSize});

	let bytesRead = 0;
	stream.on('end', () => {
	    console.log(`Flushing on end after ${bytesRead} bytes`);
	    mp4boxfile.flush();
	});
        stream.on('data', (chunk) => {
            const arrayBuffer = new Uint8Array(chunk).buffer;
            arrayBuffer.fileStart = bytesRead;
            mp4boxfile.appendBuffer(arrayBuffer);
            bytesRead += chunk.length;
        });
	stream.resume();

    });
}

/**
 * Reads data directly. Only works on smaller files.
 * @param filename
 */
function readData(filename) {
    return new Promise((resolve, reject) => {
        const mp4boxfile = MP4Box.createFile();
        mp4boxfile.onError = function (e) {
            reject(e);
        };
        mp4boxfile.onReady = function (videoData) {
            resolve({ videoData, mp4boxfile});
        };
        const arrayBuffer = new Uint8Array(fs.readFileSync(filename)).buffer;
        arrayBuffer.fileStart = 0;
        mp4boxfile.appendBuffer(arrayBuffer);
    });
}


//Will convert the final uint8Array to buffer
//https://stackoverflow.com/a/12101012/3362074
function toBuffer(ab) {
    let buf = Buffer.alloc(ab.byteLength);
    let view = new Uint8Array(ab);
    for (let i = 0; i < buf.length; ++i) {
        buf[i] = view[i];
    }
    return buf;
}

function readSampleMetadata(videoData) {
    let trackId, nb_samples, start, frameDuration;

    for (let i = 0; i < videoData.tracks.length; i++) {
	//Find the metadata track. Collect Id and number of samples
	if (videoData.tracks[i].codec === 'gpmd') {
	    trackId = videoData.tracks[i].id;
	    nb_samples = videoData.tracks[i].nb_samples;
	    start = videoData.tracks[i].created;
	} else if (videoData.tracks[i].type === 'video') {
	    const vid = videoData.tracks[i];
	    //Deduce framerate from video track
	    frameDuration = vid.movie_duration / vid.movie_timescale / vid.nb_samples;
        }
    }

    return {trackId, nb_samples, start, frameDuration}
}

class SamplesAnalyser {
    constructor(sampleMetadata) {
	this.sampleMetadata = sampleMetadata;
    }

    onSamples = (id, user, samples) => {
	const totalSamples = samples.reduce(function (acc, cur) {
	    return acc + cur.size;
	}, 0);

	//Store them in Uint8Array
	const uintArr = new Uint8Array(totalSamples);

	const outputSamples = [];
	let runningCount = 0;
	samples.forEach(function (sample) {
	    outputSamples.push({cts: sample.cts, duration: sample.duration});
	    uintArr.set(sample.data, runningCount);
	    runningCount += sample.size;
	});
	const rawData = toBuffer(uintArr);

	const timing = {'samples': outputSamples, ...this.sampleMetadata};

	this.data = {rawData, timing};
    }

}

function extractVideoData(videoData, mp4boxfile) {
    const sampleMetadata = readSampleMetadata(videoData);
    if (sampleMetadata.trackId == null) {
	return Promise.reject('Track not found');
    }

    //Request the track
    mp4boxfile.setExtractionOptions(sampleMetadata.trackId, null, {
	nbSamples: sampleMetadata.nb_samples
    });

    const proc = new SamplesAnalyser(sampleMetadata);
    mp4boxfile.onSamples = proc.onSamples;

    // Start doesn't fire onSamples unless sample finishes in the current block
    while (!proc.data) {
	mp4boxfile.start();
    }

    return Promise.resolve(proc.data);
}

async function extractInfo(input, output) {
    console.log(`Opening ${input}`);
    const {videoData, mp4boxfile} = await streamData(input);
    console.log(`Read stream data from ${path.basename(input)}`);
    const extracted = await extractVideoData(videoData, mp4boxfile);
    console.log(`Extracted information for ${path.basename(input)}`);
    const telemetry = await goproTelemetry(extracted, {'stream': ['ACCL','GYRO','GPS5','GPSF','GPSP','SROT'], 
                'repeatSticky': true, 'promisify': true, 'ellisoid': true, 'geoidHeight':true});
    console.log(`Generated telemetry for ${path.basename(input)}`);
    fs.writeFileSync(output, JSON.stringify(telemetry));
    console.log(`Telemetry saved as JSON to ${path.basename(output)}`);
}

module.exports = {
    streamData,
    readData,
    extractVideoData,
    extractInfo,
};