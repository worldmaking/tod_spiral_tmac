{
  "targets": [
    {
      "target_name": "an",
      "sources": [ "src/an.cpp" ],
      'include_dirs': [
        'src', 'include'
      ],
      'cflags':[],
      'conditions': [
        ['OS=="mac"',
          {
            'libraries': [],
            'include_dirs': [],
            'library_dirs': [],
            'xcode_settings': {
              'MACOSX_DEPLOYMENT_TARGET': '10.13',
              'OTHER_CFLAGS': [
                "-Wno-unused-but-set-variable","-Wno-unused-parameter","-Wno-unused-variable","-Wno-int-to-void-pointer-cast"
              ],
            }
          }
        ],
        ['OS=="linux"', {
          'libraries': []
          }
        ],
        ['OS=="win"',
          {
            'include_dirs': [],
            'library_dirs': [],
            'libraries': [],
            'defines' : [
              'WIN32_LEAN_AND_MEAN',
              'VC_EXTRALEAN'
            ],
            'msvs_settings' : {
              'VCCLCompilerTool' : {
                'AdditionalOptions' : []
              },
              'VCLinkerTool' : {
                'AdditionalOptions' : ['/OPT:REF','/OPT:ICF','/LTCG']
              },
            },
            'copies': [
              {
                'destination': './build/<(CONFIGURATION_NAME)/',
                'files': []
              }
            ],
          }
        ],
      ],
    }
  ]
}