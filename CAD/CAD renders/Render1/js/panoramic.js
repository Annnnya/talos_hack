/*!
 * SOLIDWORKS Visualize Panoramic Image (Pre)Viewer
 * 
 * * This logic or similar would be necessary to embed a Panolens.js viewer into another site.
 * * Mobile device Sensor control likely requires https hosting.
 */

/*!
 * Panolens.js is licensed under the:
 * The MIT License (MIT)
 * Copyright (c) 2015-2019 Ray Chen
 * 
 * https://github.com/pchen66/panolens.js/blob/master/LICENSE  
 */

/*!
 * three.js is licensed under the:
 * The MIT License (MIT)
 * Copyright (c) 2010-2023 three.js authors
 * 
 * https://github.com/mrdoob/three.js/blob/dev/LICENSE
 */

(function Viewer() {
    const useStereoImagePanorama = false;
    const imageSrc = 'images/panorama.png';
    const panorama = useStereoImagePanorama ? new PANOLENS.StereoImagePanorama(imageSrc) : new PANOLENS.ImagePanorama(imageSrc);

    panorama.addEventListener('load', function () {
        viewer.getControl().rotateLeft(Math.PI / 2);
        viewer.setCameraFov(90); // Default is 60
        viewer.getControl().update();
    });
    const viewer = new PANOLENS.Viewer();
    viewer.add(panorama);
})();
