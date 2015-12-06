/*
 * GET or POST image for localization.
 */
/*
var validator = require('validator'), async = require('async'), request = require('request'), fs = require('fs'),
localizeImage = require('bindings')('localizeImage');
*/
var validator = require('validator'), async = require('async'), request = require('request'), fs = require('fs');

/*
 * GET parameters 
 *  user : ID of user
 * 	map : ID of map
 * 	image : URL of request image
 *  cx : center to restrict localization area (optional)
 *  cy : center to restrict localization area (optional)
 *  cz : center to restrict localization area (optional)
 *  radius : center to restrict localization area (optional)
 * example request
 * http://localhost:3000/localize?user=1&map=office&image=https://www.google.co.jp/images/srpr/logo11w.png
 */
exports.estimateGet = function(req, res) {
	console.log("localize by GET request is called");

	var sendErrorResponse = function(code, message) {
		res.statusCode = code;
		res.setHeader("Content-Type", "application/json");
		res.write(JSON.stringify({
			message : message
		}));
		res.end();
	};
	if (!req.query.user) {
		return sendErrorResponse(404, 'User ID is not specified');
	}
	if (!req.query.map) {
		return sendErrorResponse(404, 'Map ID is not specified');
	}
	if (!req.query.image) {
		return sendErrorResponse(404, 'Image URL is not specified');
	}
	
	var imageURL = decodeURIComponent(req.query.image);
	if (!validator.isURL(imageURL)) {
		return sendErrorResponse(404, 'Image URL is not valid');
	}
	
	async.waterfall([ function(callback) {
		var requestSettings = {
			method : 'GET',
			url : imageURL,
			encoding : null
		};

		var size = 0;
		var req = request(requestSettings, function(error, response, body) {
			callback(error, body);
		}).on('data', function(chunk) {
			size += chunk.length;
			console.log('download size : ' + size);
		});
	}, function(image, callback) {
		var result;
		/*
		if (req.query.cx && req.query.cy && req.query.cz && req.query.radius) {
			result = localizeImage.localizeImageBuffer(req.query.user, req.query.map, image,
					[req.query.cx, req.query.cy, req.query.cz], req.query.radius);
		} else {
			result = localizeImage.localizeImageBuffer(req.query.user, req.query.map, image);
		}
		*/
		// result = [1, 2, 3];
		result = new Array(3);
		for (var i = 0; i < result.length; i++) {
			result[i] = Math.random() * (60 - 1) + 1;
		}
		console.log('localization result : ' + result);
		var jsonObj = {'estimate':result};
		callback(null, JSON.stringify(jsonObj));
	} ], function(err, result) {
		if (err) {
			sendErrorResponse(500, err.message);
		}
		res.setHeader("Content-Type", "application/json");
		res.write(result);
		res.end();
	});
};

/*
 * POST parameters 
 *  user : ID of user
 * 	map : ID of map
 *  image : binary image data
 *  cx : center to restrict localization area (optional)
 *  cy : center to restrict localization area (optional)
 *  cz : center to restrict localization area (optional)
 *  radius : center to restrict localization area (optional)
 */
exports.estimatePost = function(req, res) {
	console.log("localize by POST request is called");

	var sendErrorResponse = function(code, message) {
		res.statusCode = code;
		res.setHeader("Content-Type", "application/json");
		res.write(JSON.stringify({
			message : message
		}));
		res.end();
	};
	if (!req.body.user) {
		return sendErrorResponse(404, 'User ID is not specified');
	}
	if (!req.body.map) {
		return sendErrorResponse(404, 'Map ID is not specified');
	}
	if (!req.files || !req.files.image) {
		return sendErrorResponse(400, "Image data is not specified");
	}
	
	async.waterfall([ function(callback) {
		fs.readFile(req.files.image.path, function (err, data) {
			var imageName = req.files.image.name;
			if(!imageName){
				console.log("Error to load uploaded image");
				res.redirect("/");
				res.end();
			} else {
				var result;
				/*
				if (req.body.cx && req.body.cy && req.body.cz && req.body.radius) {
					result = localizeImage.localizeImageBuffer(req.body.user, req.body.map, data, 
							[req.body.cx, req.body.cy, req.body.cz], req.body.radius);					
				} else {
					result = localizeImage.localizeImageBuffer(req.body.user, req.body.map, data);
				}
				*/
				result = new Array(3);
				for (var i = 0; i < result.length; i++) {
    				result[i] = randomIntInc(1, 60)
				}
				// result = [1, 2, 3];
				console.log('localization result : ' + result);
				var jsonObj = {'estimate':result};
				callback(null, JSON.stringify(jsonObj));
			}
		});
	} ], function(err, result) {
		if (err) {
			sendErrorResponse(500, err.message);
		}
		res.setHeader("Content-Type", "application/json");
		res.write(result);
		res.end();
	});
};