/*
 * GET or POST beacon for localization.
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
 * http://localhost:3000/beacon?user=1&map=office&beacon=2,1,-90,2,-86&timestamp=190
 */
exports.estimateGet = function(req, res) {
	console.log("beacon localize by GET request is called");

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
	if (!req.query.beacon) {
		return sendErrorResponse(404, 'Beacon data is not specified');
	}
	if (!req.query.timestamp) {
		return sendErrorResponse(404, 'Timestamp data is not specified');
	}
	
	async.waterfall([ function(callback) {
		// req.query.beacon;
		// console.log(req.query.beacon);
		callback(null, req.query.beacon);

	}, function(beacon, callback) {
		console.log(beacon);

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
	if (!req.files || !req.files.beacon) {
		return sendErrorResponse(400, "Beacon data is not specified");
	}
	
	async.waterfall([ function(callback) {
		fs.readFile(req.files.beacon.path, function (err, data) {
			var dataName = req.files.beacon.name;
			if(!dataName){
				console.log("Error to load uploaded beacon");
				res.redirect("/");
				res.end();
			} else {
				console.log("Received beacon " + dataName);
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
					result[i] = Math.random() * (100 - 90) + 90;
				}
				// result = [1, 2, 3];
				console.log('localization result : ' + result);
				var jsonObj = {'estimate':result, 'timestamp':dataName};
				// var jsonObj = [{'estimate':result}, {'filename':imageName}];
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