//---------------------------------------------------------------------------

#include "metadata/reader.hxx"

MetadataReader Metadata;

SessionMetadata MetadataReader::read_metadata_from_file(const std::filesystem::path& metadata_file) {
	reader_.read(metadata_file);
	const SessionMetadata session_data_{
		reader_.get_string("name").value(),
		reader_.get_string("description"),
		reader_.get_string("display_name"),
		reader_.get_string("rly_file").value(),
		reader_.get_list("ttb_files").value(),
		reader_.get_list("ssn_files"),
		reader_.get_string("doc_file").value(),
		reader_.get_list("img_files"),
		reader_.get_list("graphics_files"),
		iso::from_string(reader_.get_string("country_code").value()),
		reader_.get_integer("year"),
		static_cast<bool>(reader_.get_boolean("factual").value()),
		reader_.get_integer("difficulty"),
		reader_.get_string("author").value(),
		reader_.get_list("contributors"),
		reader_.get_version("version").value(),
		reader_.get_date("release_date").value(),
		reader_.get_version("minimum_required"),
		(reader_.get_string("signal_position").has_value()) ? ((reader_.get_string("signal_position").value() == "left") ? SignalPosition::Left : SignalPosition::Right) : SignalPosition::Left
	};
	return session_data_;
}

void MetadataReader::read_local_simulation(const std::string& simulation_name) {

}
