#include "shared.h"
#include "al/al_isosurface.h"

#include "node-api-helpers.h"

napi_value initialize(napi_env env, napi_callback_info info) {
	// napi_status status = napi_ok;
	// napi_value args[1];
	// size_t argc = checkArgCount(env, info, args, 1, 0);

	// bool enable = getBool(env, args[0]);

	// if (!session) session = XRSession::create(env);

	// bool ok = true;
	// if (enable) {
	// 	ok = session->hmd.connect();
	// } else {
	// 	session->hmd.disconnect();
	// 	session = nullptr;
	// }
	// napi_value result_value;
	// status = napi_create_uint32(env, ok ? 1 : 0, &result_value);
	// return (status == napi_ok) ? result_value : nullptr;
	return nullptr;
}


napi_value init(napi_env env, napi_value exports) {
	napi_status status;
	napi_property_descriptor properties[] = {
		{ "initialize", 0, initialize, 0, 0, 0, napi_default, 0 },

		// { "connect", 0, connect, 0, 0, 0, napi_default, 0 },
		// { "getTextureWidth", 0, getTextureWidth, 0, 0, 0, napi_default, 0 },
		// { "getTextureHeight", 0, getTextureHeight, 0, 0, 0, napi_default, 0 },
		// { "getProjection", 0, getProjection, 0, 0, 0, napi_default, 0 },
		// { "getView", 0, getView, 0, 0, 0, napi_default, 0 },
		// { "inputSources", 0, inputSources, 0, 0, 0, napi_default, 0 },
		// { "update", 0, update, 0, 0, 0, napi_default, 0 },
		// { "submit", 0, submit, 0, 0, 0, napi_default, 0 },
		// { "getModelNames", 0, getModelNames, 0, 0, 0, napi_default, 0 }
	};
	status = napi_define_properties(env, exports, sizeof(properties)/sizeof(napi_property_descriptor), properties);
	//assert(status == napi_ok);
	return exports;
}
NAPI_MODULE(NODE_GYP_MODULE_NAME, init)