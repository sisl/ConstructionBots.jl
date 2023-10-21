using DataFrames
using TOML
using CSV

function run_me()
    # Define the path to your results directory
    results_dir = "results/results"

    # Initialize an empty DataFrame to store the results
    results_df = DataFrame()

    param_names = ["assignment", "rvo", "tangent_bug", "dispersion"]

    # Loop through each subdirectory in the results directory
    for project_name in readdir(results_dir)
        if isdir(joinpath(results_dir, project_name))
            for subdir in readdir(joinpath(results_dir, project_name))
                # Split the subdirectory name into its constituent parameters
                params = split(subdir, "_")

                # If extension is jld2, continue
                if subdir[end-3:end] == "jld2"
                    continue
                end

                if isdir(joinpath(results_dir, project_name, subdir))
                    for file in readdir(joinpath(results_dir, project_name, subdir); join=true)
                        if file[end-3:end] != "toml"
                            continue
                        end

                        # get file from path
                        file_name = split(file, "/")[end]
                        file_name_split = split(file_name, "_")
                        file_date = file_name_split[1]
                        file_time = file_name_split[2]

                        # Parse the TOML file and convert it to a DataFrame
                        data = DataFrame(TOML.parsefile(joinpath(pwd(), file)))

                        data[:, "project_name"] .= project_name
                        data[:, "date"] .= file_date
                        data[:, "time"] .= file_time

                        if all(col in names(data) for col in ["rvo_flag", "dispersion_flag", "tangent_bug_flag"])
                            col_t = data[:, "rvo_flag"] .&& data[:, "dispersion_flag"] .&& data[:, "tangent_bug_flag"]
                            col_f = .!(data[:, "rvo_flag"]) .&& .!data[:, "dispersion_flag"] .&& .!data[:, "tangent_bug_flag"]
                            data[:, "col_t"] = col_t
                            data[:, "col_f"] = col_f
                        end

                        # Append the DataFrame to the results DataFrame
                        append!(results_df, data; cols=:union)
                    end
                end
            end
        end
    end

    select!(results_df, Not(:Optimizer))

    dropmissing!(results_df, [:AssigmentTime, :ConfigTransportUnitsTime])

    transform!(results_df, :ExecutionRuntime => ByRow(x -> coalesce(x, Inf)); renamecols=false)
    transform!(results_df, :Makespan => ByRow(x -> coalesce(x, Inf)); renamecols=false)

    # Define the desired column order
    new_order = ["project_name", "assignment_mode", "date", "time", "col_t", "col_f", "numrobots", "ConfigTransportUnitsTime", "StagingPlanTime", "AssigmentTime", "OptimisticMakespan", "Makespan", "ExecutionRuntime"]

    # Columns not in new_order
    other_cols = [n for n in names(results_df) if n ∉ new_order]

    # Combine new_order and other_cols
    new_order = vcat(new_order, other_cols)

    # Select the columns in the desired order
    results_df_no = results_df[:, new_order]

    # Print the resulting DataFrame
    println(results_df_no)

    CSV.write("results/results.csv", results_df_no)
end

run_me()
