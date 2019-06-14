properties ([[$class: 'ParametersDefinitionProperty', parameterDefinitions: [
  [$class: 'StringParameterDefinition', name: 'mbed_os_revision', defaultValue: '', description: 'Revision of mbed-os to build. To access mbed-os PR use format "pull/PR number/head"'],
  [$class: 'BooleanParameterDefinition', name: 'smoke_test', defaultValue: true, description: 'Runs HW smoke tests on Cellular devices']
  ]]])

if (env.MBED_OS_REVISION == null) {
  echo 'First run in this branch, using default parameter values'
  env.MBED_OS_REVISION = ''
}
if (env.MBED_OS_REVISION == '') {
  echo 'Using mbed OS revision from mbed-os.lib'
} else {
  echo "Using given mbed OS revision: ${env.MBED_OS_REVISION}"
  if (env.MBED_OS_REVISION.matches('pull/\\d+/head')) {
    echo "Revision is a Pull Request"
  }
}

// Map RaaS instances to corresponding test suites
def raas = [
  "cellular_smoke_nrf52840_dk.json": "auli"
]

// Supported Modems
def targets = [
  //"MTB_MTS_DRAGONFLY",
  //"UBLOX_C030_U201",
  //"MTB_ADV_WISE_1570"
  "NRF52840_DK"
]

// Map toolchains to compilers
def toolchains = [
  ARM: "armcc",
  GCC_ARM: "arm-none-eabi-gcc",
  IAR: "iar_arm"
  ]

def stepsForParallel = [:]

// Jenkins pipeline does not support map.each, we need to use oldschool for loop
for (int i = 0; i < targets.size(); i++) {
  for(int j = 0; j < toolchains.size(); j++) {
    def target = targets.get(i)
    def toolchain = toolchains.keySet().asList().get(j)
    def compilerLabel = toolchains.get(toolchain)
    def stepName = "${target} ${toolchain}"

    stepsForParallel[stepName] = buildStep(target, compilerLabel, toolchain)
  }
}

def parallelRunSmoke = [:]

// Need to compare boolean against string value
if (params.smoke_test == true) {
  echo "Running smoke tests"
  // Generate smoke tests based on suite amount
  for(int i = 0; i < raas.size(); i++) {
    def suite_to_run = raas.keySet().asList().get(i)
    def raasName = raas.get(suite_to_run)

    // Parallel execution needs unique step names. Remove .json file ending.
    def smokeStep = "${raasName} ${suite_to_run.substring(0, suite_to_run.indexOf('.'))}"
    parallelRunSmoke[smokeStep] = run_smoke(raasName, suite_to_run, toolchains, targets)
  }
} else {
  echo "Skipping smoke tests"
}

timestamps {
  parallel stepsForParallel
  parallel parallelRunSmoke
}

def buildStep(target, compilerLabel, toolchain) {
  return {
    stage ("${target}_${compilerLabel}") {
      node ("${compilerLabel}") {
        deleteDir()
        dir("mbed-os-example-cellular") {
          checkout scm
          def config_file = "mbed_app.json"

          // Configurations for different targets

          if ("${target}" == "MTB_ADV_WISE_1570") {
            execute("sed -i 's/\"lwip.ppp-enabled\": true,/\"lwip.ppp-enabled\": false,/' ${config_file}")
            execute("sed -i 's/\"platform.default-serial-baud-rate\": 115200,/\"platform.default-serial-baud-rate\": 9600,/' ${config_file}")
          }

          // Required due to network tester
          if ("${target}" == "NRF52840_DK") {
            execute("sed -i 's/\"cellular.radio-access-technology\": 3,/\"cellular.radio-access-technology\": 9,/' ${config_file}")
          }

          // Set mbed-os to revision received as parameter
          execute ("mbed deploy --protocol ssh")
          if (env.MBED_OS_REVISION != '') {
            dir("mbed-os") {
              if (env.MBED_OS_REVISION.matches('pull/\\d+/head')) {
                // Use mbed-os PR and switch to branch created
                execute("git fetch origin ${env.MBED_OS_REVISION}:_PR_")
                execute("git checkout _PR_")
              } else {
                execute ("git checkout ${env.MBED_OS_REVISION}")
              }
            }
          }

          execute ("mbed compile --build out/${target}_${toolchain}/ -m ${target} -t ${toolchain} -c --app-config ${config_file} \
                   |tee ${target}_${toolchain}_build.txt")

          execute ("cat ${target}_${toolchain}_build.txt |grep \"Total Static RAM memory\" |awk '{print \$8}' |cut -f1 -d'(' > ${target}_${toolchain}_RAM.log")
          execute("cat ${target}_${toolchain}_build.txt |grep \"Total Flash memory\" |awk '{print \$7}' |cut -f1 -d'(' > ${target}_${toolchain}_ROM.log")

        }
        if ("${target}" == "MTB_ADV_WISE_1570" || "${target}" == "NRF52840_DK") {
          stash name: "${target}_${toolchain}", includes: '**/mbed-os-example-cellular.hex'
          archive '**/mbed-os-example-cellular.hex, **/*.log'
        }
        else {
          stash name: "${target}_${toolchain}", includes: '**/mbed-os-example-cellular.bin'
          archive '**/mbed-os-example-cellular.bin'
        }
        step([$class: 'WsCleanup'])
      }
    }
  }
}

def run_smoke(raasName, suite_to_run, toolchains, targets) {
  return {
    env.RAAS_USERNAME = "ci"
    env.RAAS_PASSWORD = "ci"
    // Remove .json from suite name
    def suiteName = suite_to_run.substring(0, suite_to_run.indexOf('.'))
    stage ("smoke_${raasName}_${suiteName}") {
      //node is actually the type of machine, i.e., mesh-test boild down to linux
      node ("linux") {
        deleteDir()
        dir("mbed-clitest") {
          git "git@github.com:ARMmbed/mbed-clitest.git"
          execute("git checkout ${env.LATEST_CLITEST_STABLE_REL}")
          dir("mbed-clitest-suites") {
            git "git@github.com:ARMmbed/mbed-clitest-suites.git"
            execute("git submodule update --init --recursive")
            execute("git all checkout master")
            dir("cellular") {
              execute("git checkout master")
            }
          }
                
          for (int i = 0; i < targets.size(); i++) {
            for(int j = 0; j < toolchains.size(); j++) {
              def target = targets.get(i)
              def toolchain = toolchains.keySet().asList().get(j)
              unstash "${target}_${toolchain}"
            }
          }     
          execute("python clitest.py --suitedir mbed-clitest-suites/suites/ --suite ${suite_to_run} --type hardware --reset \
                  --raas https://${raasName}.mbedcloudtesting.com:443 --tcdir mbed-clitest-suites/cellular --raas_queue --raas_queue_timeout 3600 \
                  --raas_share_allocs --failure_return_value -v -w --log log_${raasName}_${suiteName}")
          archive "log_${raasName}_${suiteName}/**/*"
        }
      }
    }
  }
}