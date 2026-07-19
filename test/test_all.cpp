#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

const fs::path sln_path = fs::current_path().parent_path();
const fs::path aera_exe_path = sln_path / "Release" / "AERA.exe";
const fs::path prj_path = sln_path / "test";
const fs::path test_path = prj_path / "tests";
const fs::path settings_template_path = test_path / "settings_template.xml";
const fs::path settings_filled_path = test_path / "settings_filled.xml";
const fs::path output_log_path = test_path / "AERA_output.log";
const fs::path decompiled_objects_path = sln_path / "Release" / "decompiled_objects.txt";

namespace AeraTests { 

    class AeraTest : public ::testing::TestWithParam<fs::path> {
    protected:
        // optional setup/teardown
        void SetUp() override {}
        void TearDown() override {}
    };

    TEST_P(AeraTest, Test) {
        const fs::path inputFile = GetParam();
        GTEST_LOG_(INFO) << "Testing file: " << inputFile.filename();

        if (!fs::exists(settings_template_path)) {
            FAIL() << "Template XML file not found: " << settings_template_path;
        }

        // Load XML template (assuming template is stored as a string or file)
        std::ifstream templateFile(settings_template_path);
        std::string xmlContent((std::istreambuf_iterator<char>(templateFile)),
            std::istreambuf_iterator<char>());

        // Get path relative to test_path
        fs::path relPath = fs::relative(inputFile, test_path);

        // Convert to string with forward slashes (if needed for XML)
        std::string replacement = relPath.generic_string();

        // Replace {{SOURCE_FILE_NAME}} in XML template
        std::string toReplace = "{{SOURCE_FILE_NAME}}";
        size_t pos = xmlContent.find(toReplace);
        if (pos != std::string::npos) {
            xmlContent.replace(pos, toReplace.length(), replacement);
        }
        else {
            FAIL() << "Failed to find source_file_name in XML template";
        }

        // Replace {{WHATEVER}} in XML template
        // first, the defaults
        int duration = 1000;
        int base_period = 50000;
        int reduction_core_count = 0;
        int time_core_count = 0;
        int max_sim_time_horizon = 50000;

        // parse file once
        std::ifstream input(inputFile);
        if (input) {
            std::string line;

            const std::string p_duration = "; run_time=";
            const std::string p_base = "; base_period=";
            const std::string p_max = "; max_sim_time_horizon=";
            const std::string p_reduction = "; reduction_core_count=";
            const std::string p_time = "; time_core_count=";

            while (std::getline(input, line)) {
                if (line.rfind(p_duration, 0) == 0) {
                    duration = std::stoi(line.substr(p_duration.size()));
                }
                else if (line.rfind(p_base, 0) == 0) {
                    base_period = std::stoi(line.substr(p_base.size()));
                }
                else if (line.rfind(p_max, 0) == 0) {
                    max_sim_time_horizon = std::stoi(line.substr(p_max.size()));
                }
                else if (line.rfind(p_reduction, 0) == 0) {
                    reduction_core_count = std::stoi(line.substr(p_reduction.size()));
                }
                else if (line.rfind(p_time, 0) == 0) {
                    time_core_count = std::stoi(line.substr(p_time.size()));
                }
            }
        }

        // helper lambda
        auto replaceKey = [&](const std::string& key, int value) {
            size_t pos = xmlContent.find(key);
            if (pos != std::string::npos) {
                xmlContent.replace(pos, key.length(), std::to_string(value));
            }
            else {
                FAIL() << "Failed to find " << key << " in XML template";
            }
        };

        // replace all
        replaceKey("{{DURATION}}", duration);
        replaceKey("{{BASE_PERIOD}}", base_period);
        replaceKey("{{REDUCTION_CORE_COUNT}}", reduction_core_count);
        replaceKey("{{TIME_CORE_COUNT}}", time_core_count);
        replaceKey("{{MAX_SIM_TIME_HORIZON}}", max_sim_time_horizon);

        // Write modified XML to temp file for this test run
        std::ofstream outFile(settings_filled_path);
        outFile << xmlContent;
        outFile.close();

        // Clear previous contents
        std::ofstream ofs(output_log_path, std::ofstream::trunc);
        ofs.close();

        // Check if this is a negative test
        bool expectDecompilation = inputFile.filename().string().rfind("testFail", 0) != 0;

        // Run AERA and redirect stdout/stderr to log file
        std::string command = "cmd /C \"\""
            + aera_exe_path.string() + "\" \""
            + settings_filled_path.string() + "\" > \""
            + output_log_path.string() + "\" 2>&1\"";
        std::string x = command.c_str();
        int result = std::system(command.c_str());

        // Read output log
        std::ifstream logFile(output_log_path);
        ASSERT_TRUE(logFile) << "Cannot open AERA output log: " << output_log_path;

        std::string logContent((std::istreambuf_iterator<char>(logFile)),
            std::istreambuf_iterator<char>());

        // Load expected lines
        fs::path expectedTxtPath = inputFile;
        expectedTxtPath.replace_extension(".txt");
        ASSERT_TRUE(fs::exists(expectedTxtPath)) << "Expected text file not found: " << expectedTxtPath;

        std::ifstream expectedFile(expectedTxtPath);
        ASSERT_TRUE(expectedFile) << "Cannot open expected text file: " << expectedTxtPath;

        // Collect expected lines
        std::vector<std::string> expectedLines;
        std::string line;
        while (std::getline(expectedFile, line)) {
            if (!line.empty() && line[0] != ';') { // remove empty lines and comments from expected lines
                expectedLines.push_back(line);
            }
        }

        if (expectDecompilation) {
            // Positive test: DECOMPILATION must appear
            ASSERT_NE(logContent.find("DECOMPILATION"), std::string::npos)
                << "AERA call failed, 'DECOMPILATION' not found in output. See " << output_log_path;

            // Check expected lines in decompiled file
            std::ifstream decompiledFile(decompiled_objects_path);
            ASSERT_TRUE(decompiledFile) << "Cannot open decompiled objects file: " << decompiled_objects_path;

            std::string decompiledContent((std::istreambuf_iterator<char>(decompiledFile)),
                std::istreambuf_iterator<char>());

            for (const auto& expectedLine : expectedLines) {
                if (decompiledContent.find(expectedLine) == std::string::npos) {
                    FAIL() << "Expected line not found in decompiled output: " << expectedLine;
                }
            }
        }
        else {
            // Negative test: DECOMPILATION must NOT appear
            ASSERT_EQ(logContent.find("DECOMPILATION"), std::string::npos)
                << "Unexpected decompilation for negative test: " << inputFile;

            // Check expected lines in log instead
            for (const auto& expectedLine : expectedLines) {
                // Skip lines starting with ';'
                if (!expectedLine.empty() && expectedLine.front() == ';') {
                    continue;
                }

                if (logContent.find(expectedLine) == std::string::npos) {
                    FAIL() << "Expected line not found in log output: " << expectedLine;
                }
            }
        }
    }

    // TODO: some of the tests for the technical report can fail seemingly at random. 
    // this makes sure they aren't executed.
    static const std::vector<std::string> blacklist = {
        "mk",  // marker classes
        "performance-counters" 
    };

    // Helper: recursively gather all .replicode files in a folder
    std::vector<fs::path> GetReplicodeFiles(const fs::path& folder, bool useBlacklist) {
        std::vector<fs::path> files;
        if (!fs::exists(folder)) return files;

        for (const auto& entry : fs::recursive_directory_iterator(folder)) {
            if (entry.is_regular_file() && entry.path().extension() == ".replicode") {
                std::string name = entry.path().parent_path().filename().string() + "_" +
                    entry.path().stem().string();

                bool blacklisted = false;
                if (useBlacklist) {
                    for (const auto& bad : blacklist) {
                        if (name.find(bad) != std::string::npos) {
                            blacklisted = true;
                            break;
                        }
                    }
                }

                if (!blacklisted)
                    files.push_back(entry.path());
            }
        }
        return files;
    }


    // EXISTING
    /*INSTANTIATE_TEST_SUITE_P(
        EXISTING,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "existing")),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    )

    // TECHNICAL REPORT - MAIN
    INSTANTIATE_TEST_SUITE_P(
        REPORT_MAIN,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "technical-report/main", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // TECHNICAL REPORT - ANNEX 1
    INSTANTIATE_TEST_SUITE_P(
        REPORT_ANNEX_1,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "technical-report/annex1", true)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // TECHNICAL REPORT - ANNEX 2
    INSTANTIATE_TEST_SUITE_P(
        REPORT_ANNEX_2,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "technical-report/annex2", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // FREE TESTS
    INSTANTIATE_TEST_SUITE_P(
        FREE_TESTS,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "free-tests", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );*/

    // INTELLIGENCE REQUIREMENTS - 1 - EMBODIMENT
    INSTANTIATE_TEST_SUITE_P(
        1_EMBODIMENT,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/1-embodiment", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 2 - CONSTRUCTIVISM
    INSTANTIATE_TEST_SUITE_P(
        2_CONSTRUCTIVISM,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/2-constructivism", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 3 - CUMULATIVE LEARNING
    INSTANTIATE_TEST_SUITE_P(
        3_CUMULATIVE_LEARNING,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/3-cumulative-learning", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 4 - KNOWLEDGE OF CAUSES
    INSTANTIATE_TEST_SUITE_P(
        4_KNOWLEDGE_OF_CAUSES,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/4-knowledge-of-causes", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 5 - NON-AXIOMATIC REASONING
    INSTANTIATE_TEST_SUITE_P(
        5_NON_AXIOMATIC_REASONING,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/5-non-axiomatic-reasoning", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 6 - GENERALITY
    INSTANTIATE_TEST_SUITE_P(
        6_GENERALITY,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/6-generality", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 7 - TRANSVERSAL FUNCTIONS
    INSTANTIATE_TEST_SUITE_P(
        7_TRANSVERSAL_FUNCTIONS,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/7-transversal-functions", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 8 - REFLECTION
    INSTANTIATE_TEST_SUITE_P(
        8_REFLECTION,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/8-reflection", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );

    // INTELLIGENCE REQUIREMENTS - 9 - ROBUSTNESS
    INSTANTIATE_TEST_SUITE_P(
        9_ROBUSTNESS,
        AeraTest,
        ::testing::ValuesIn(GetReplicodeFiles(test_path / "intelligence-requirements/9-robustness", false)),
        [](const ::testing::TestParamInfo<fs::path>& info) {
            std::string name = info.param.parent_path().filename().string() + "_" +
                info.param.filename().stem().string();
            for (auto& c : name) if (!isalnum(c)) c = '_';
            return name;
        }
    );
}