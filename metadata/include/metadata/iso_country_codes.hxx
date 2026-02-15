#pragma once

#include <string_view>
#include <stdexcept>

namespace iso {

#define ISO_COUNTRY_LIST(X) \
    X(Afghanistan, "AF") \
    X(Albania, "AL") \
    X(Algeria, "DZ") \
    X(Andorra, "AD") \
    X(Angola, "AO") \
    X(Antigua_And_Barbuda, "AG") \
    X(Argentina, "AR") \
    X(Armenia, "AM") \
    X(Australia, "AU") \
    X(Austria, "AT") \
    X(Azerbaijan, "AZ") \
    X(Bahamas, "BS") \
    X(Bahrain, "BH") \
    X(Bangladesh, "BD") \
    X(Barbados, "BB") \
    X(Belarus, "BY") \
    X(Belgium, "BE") \
    X(Belize, "BZ") \
    X(Benin, "BJ") \
    X(Bhutan, "BT") \
    X(Bolivia, "BO") \
    X(Bosnia_And_Herzegovina, "BA") \
    X(Botswana, "BW") \
    X(Brazil, "BR") \
    X(Brunei, "BN") \
    X(Bulgaria, "BG") \
    X(Burkina_Faso, "BF") \
    X(Burundi, "BI") \
    X(Cabo_Verde, "CV") \
    X(Cambodia, "KH") \
    X(Cameroon, "CM") \
    X(Canada, "CA") \
    X(Central_African_Republic, "CF") \
    X(Chad, "TD") \
    X(Chile, "CL") \
    X(China, "CN") \
    X(Colombia, "CO") \
    X(Comoros, "KM") \
    X(Congo, "CG") \
    X(Congo_Democratic_Republic, "CD") \
    X(Costa_Rica, "CR") \
    X(Cote_DIvoire, "CI") \
    X(Croatia, "HR") \
    X(Cuba, "CU") \
    X(Cyprus, "CY") \
    X(Czechia, "CZ") \
    X(Denmark, "DK") \
    X(Djibouti, "DJ") \
    X(Dominica, "DM") \
    X(Dominican_Republic, "DO") \
    X(Ecuador, "EC") \
    X(Egypt, "EG") \
    X(El_Salvador, "SV") \
    X(Equatorial_Guinea, "GQ") \
    X(Eritrea, "ER") \
    X(Estonia, "EE") \
    X(Eswatini, "SZ") \
    X(Ethiopia, "ET") \
    X(Fiji, "FJ") \
    X(Finland, "FI") \
    X(France, "FR") \
    X(Gabon, "GA") \
    X(Gambia, "GM") \
    X(Georgia, "GE") \
    X(Germany, "DE") \
    X(Ghana, "GH") \
    X(Greece, "GR") \
    X(Grenada, "GD") \
    X(Guatemala, "GT") \
    X(Guinea, "GN") \
    X(Guinea_Bissau, "GW") \
    X(Guyana, "GY") \
    X(Haiti, "HT") \
    X(Honduras, "HN") \
    X(Hungary, "HU") \
    X(Iceland, "IS") \
    X(India, "IN") \
    X(Indonesia, "ID") \
    X(Iran, "IR") \
    X(Iraq, "IQ") \
    X(Ireland, "IE") \
    X(Israel, "IL") \
    X(Italy, "IT") \
    X(Jamaica, "JM") \
    X(Japan, "JP") \
    X(Jordan, "JO") \
    X(Kazakhstan, "KZ") \
    X(Kenya, "KE") \
    X(Kiribati, "KI") \
    X(Korea_North, "KP") \
    X(Korea_South, "KR") \
    X(Kuwait, "KW") \
    X(Kyrgyzstan, "KG") \
    X(Laos, "LA") \
    X(Latvia, "LV") \
    X(Lebanon, "LB") \
    X(Lesotho, "LS") \
    X(Liberia, "LR") \
    X(Libya, "LY") \
    X(Liechtenstein, "LI") \
    X(Lithuania, "LT") \
    X(Luxembourg, "LU") \
    X(Madagascar, "MG") \
    X(Malawi, "MW") \
    X(Malaysia, "MY") \
    X(Maldives, "MV") \
    X(Mali, "ML") \
    X(Malta, "MT") \
    X(Marshall_Islands, "MH") \
    X(Mauritania, "MR") \
    X(Mauritius, "MU") \
    X(Mexico, "MX") \
    X(Micronesia, "FM") \
    X(Moldova, "MD") \
    X(Monaco, "MC") \
    X(Mongolia, "MN") \
    X(Montenegro, "ME") \
    X(Morocco, "MA") \
    X(Mozambique, "MZ") \
    X(Myanmar, "MM") \
    X(Namibia, "NA") \
    X(Nauru, "NR") \
    X(Nepal, "NP") \
    X(Netherlands, "NL") \
    X(New_Zealand, "NZ") \
    X(Nicaragua, "NI") \
    X(Niger, "NE") \
    X(Nigeria, "NG") \
    X(North_Macedonia, "MK") \
    X(Norway, "NO") \
    X(Oman, "OM") \
    X(Pakistan, "PK") \
    X(Palau, "PW") \
    X(Panama, "PA") \
    X(Papua_New_Guinea, "PG") \
    X(Paraguay, "PY") \
    X(Peru, "PE") \
    X(Philippines, "PH") \
    X(Poland, "PL") \
    X(Portugal, "PT") \
    X(Qatar, "QA") \
    X(Romania, "RO") \
    X(Russia, "RU") \
    X(Rwanda, "RW") \
    X(Saint_Kitts_And_Nevis, "KN") \
    X(Saint_Lucia, "LC") \
    X(Saint_Vincent_And_The_Grenadines, "VC") \
    X(Samoa, "WS") \
    X(San_Marino, "SM") \
    X(Sao_Tome_And_Principe, "ST") \
    X(Saudi_Arabia, "SA") \
    X(Senegal, "SN") \
    X(Serbia, "RS") \
    X(Seychelles, "SC") \
    X(Sierra_Leone, "SL") \
    X(Singapore, "SG") \
    X(Slovakia, "SK") \
    X(Slovenia, "SI") \
    X(Solomon_Islands, "SB") \
    X(Somalia, "SO") \
    X(South_Africa, "ZA") \
    X(South_Sudan, "SS") \
    X(Spain, "ES") \
    X(Sri_Lanka, "LK") \
    X(Sudan, "SD") \
    X(Suriname, "SR") \
    X(Sweden, "SE") \
    X(Switzerland, "CH") \
    X(Syria, "SY") \
    X(Tajikistan, "TJ") \
    X(Tanzania, "TZ") \
    X(Thailand, "TH") \
    X(Timor_Leste, "TL") \
    X(Togo, "TG") \
    X(Tonga, "TO") \
    X(Trinidad_And_Tobago, "TT") \
    X(Tunisia, "TN") \
    X(Turkey, "TR") \
    X(Turkmenistan, "TM") \
    X(Tuvalu, "TV") \
    X(Uganda, "UG") \
    X(Ukraine, "UA") \
    X(United_Arab_Emirates, "AE") \
    X(United_Kingdom, "GB") \
    X(United_States, "US") \
    X(Uruguay, "UY") \
    X(Uzbekistan, "UZ") \
    X(Vanuatu, "VU") \
    X(Vatican_City, "VA") \
    X(Venezuela, "VE") \
    X(Vietnam, "VN") \
    X(Yemen, "YE") \
    X(Zambia, "ZM") \
    X(Zimbabwe, "ZW") \
    X(Fictional, "FN")

enum class Country {
#define ENUM_ENTRY(name, code) name,
    ISO_COUNTRY_LIST(ENUM_ENTRY)
#undef ENUM_ENTRY
};

constexpr std::string_view to_string(Country c) {
    switch (c) {
#define TO_STRING_ENTRY(name, code) case Country::name: return code;
        ISO_COUNTRY_LIST(TO_STRING_ENTRY)
#undef TO_STRING_ENTRY
    }
    return {};
}

constexpr Country from_string(std::string_view code) {
#define FROM_STRING_ENTRY(name, code_str) \
    if (code == code_str) return Country::name;

    ISO_COUNTRY_LIST(FROM_STRING_ENTRY)

#undef FROM_STRING_ENTRY
    throw std::invalid_argument("Invalid ISO country code");
}

#undef ISO_COUNTRY_LIST

} // namespace iso

