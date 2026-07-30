#include "Platform/Firewall.hpp"

#include "Core/Log.hpp"

#include <format>

#if defined(_WIN32) && __has_include(<netfw.h>)
#    define SW_HAVE_WINDOWS_FIREWALL 1
#    define WIN32_LEAN_AND_MEAN
// Lowercase deliberately, unlike the rest of the tree: MSVC does not care,
// and MinGW's headers are genuinely named in lowercase, so `<Windows.h>`
// makes the file MSVC-only for no reason at all.
#    include <windows.h>
// windows.h first, always: netfw.h and shellapi.h both depend on it.
#    include <netfw.h>
#    include <shellapi.h>
#    include <objbase.h>
#    include <string>
// ole32 / oleaut32 / shell32 are linked in Engine/CMakeLists.txt rather than
// with #pragma comment: the pragma is an MSVC extension and would silently
// leave the same code unlinkable under any other Windows toolchain.
#else
#    define SW_HAVE_WINDOWS_FIREWALL 0
#endif

namespace sw::platform
{
#if SW_HAVE_WINDOWS_FIREWALL
    namespace
    {
        constexpr const wchar_t* kRuleName = L"StarWorks (inbound UDP)";

        /// COM initialised for the duration of one call, and UNINITIALISED
        /// only if this scope is what initialised it. The renderer and the
        /// shell may already have put this thread in an apartment; taking it
        /// back out from under them would be a crash with no obvious cause.
        class ComScope
        {
        public:
            ComScope()
            {
                const HRESULT hr = ::CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
                m_owned = SUCCEEDED(hr);
                // RPC_E_CHANGED_MODE means someone else already chose a
                // different apartment. That is fine — we can still use COM,
                // we simply do not own the initialisation.
                m_usable = m_owned || hr == RPC_E_CHANGED_MODE;
            }
            ~ComScope()
            {
                if (m_owned)
                {
                    ::CoUninitialize();
                }
            }
            ComScope(const ComScope&) = delete;
            ComScope& operator=(const ComScope&) = delete;

            [[nodiscard]] bool usable() const { return m_usable; }

        private:
            bool m_owned = false;
            bool m_usable = false;
        };

        std::wstring executablePathW()
        {
            wchar_t buffer[MAX_PATH]{};
            const DWORD length = ::GetModuleFileNameW(nullptr, buffer, MAX_PATH);
            if (length == 0 || length >= MAX_PATH)
            {
                return {};
            }
            return std::wstring(buffer, length);
        }

        bool equalsIgnoringCase(const std::wstring& a, const std::wstring& b)
        {
            // Windows paths are case-insensitive, and the firewall stores
            // whatever case the rule was written with. Comparing them byte
            // for byte would miss our own rule half the time.
            return a.size() == b.size() &&
                   ::CompareStringOrdinal(a.c_str(), static_cast<int>(a.size()), b.c_str(),
                                          static_cast<int>(b.size()), TRUE) == CSTR_EQUAL;
        }

        /// True if `rule` is an enabled inbound Allow rule for UDP on this
        /// exact executable. Every one of those five conditions matters: a
        /// disabled rule, a Block rule, an outbound rule or a TCP rule all
        /// leave the datagram exactly as dropped as no rule at all.
        bool ruleCovers(INetFwRule* rule, const std::wstring& exePath)
        {
            VARIANT_BOOL enabled = VARIANT_FALSE;
            if (FAILED(rule->get_Enabled(&enabled)) || enabled == VARIANT_FALSE)
            {
                return false;
            }
            NET_FW_RULE_DIRECTION direction{};
            if (FAILED(rule->get_Direction(&direction)) || direction != NET_FW_RULE_DIR_IN)
            {
                return false;
            }
            NET_FW_ACTION action{};
            if (FAILED(rule->get_Action(&action)) || action != NET_FW_ACTION_ALLOW)
            {
                return false;
            }
            // 256 is how the firewall API spells "any protocol" — there is
            // no NET_FW_IP_PROTOCOL_ constant for it, because it is not an
            // IP protocol number. A rule that allows everything inbound for
            // this executable allows UDP, so it counts.
            constexpr LONG kProtocolAny = 256;
            LONG protocol = 0;
            if (FAILED(rule->get_Protocol(&protocol)) ||
                (protocol != NET_FW_IP_PROTOCOL_UDP && protocol != kProtocolAny))
            {
                return false;
            }

            BSTR application = nullptr;
            if (FAILED(rule->get_ApplicationName(&application)) || application == nullptr)
            {
                return false;
            }
            const bool match = equalsIgnoringCase(std::wstring(application), exePath);
            ::SysFreeString(application);
            return match;
        }
    } // namespace

    FirewallState inboundUdpState()
    {
        const std::wstring exePath = executablePathW();
        if (exePath.empty())
        {
            return FirewallState::Unknown;
        }

        const ComScope com;
        if (!com.usable())
        {
            return FirewallState::Unknown;
        }

        INetFwPolicy2* policy = nullptr;
        if (FAILED(::CoCreateInstance(__uuidof(NetFwPolicy2), nullptr, CLSCTX_INPROC_SERVER,
                                      __uuidof(INetFwPolicy2), reinterpret_cast<void**>(&policy))) ||
            policy == nullptr)
        {
            return FirewallState::Unknown;
        }

        INetFwRules* rules = nullptr;
        if (FAILED(policy->get_Rules(&rules)) || rules == nullptr)
        {
            policy->Release();
            return FirewallState::Unknown;
        }

        IUnknown* unknown = nullptr;
        IEnumVARIANT* iterator = nullptr;
        if (SUCCEEDED(rules->get__NewEnum(&unknown)) && unknown != nullptr)
        {
            unknown->QueryInterface(__uuidof(IEnumVARIANT), reinterpret_cast<void**>(&iterator));
            unknown->Release();
        }
        if (iterator == nullptr)
        {
            rules->Release();
            policy->Release();
            return FirewallState::Unknown;
        }

        FirewallState state = FirewallState::Blocked;
        VARIANT item;
        ::VariantInit(&item);
        ULONG fetched = 0;
        while (state == FirewallState::Blocked && iterator->Next(1, &item, &fetched) == S_OK &&
               fetched == 1)
        {
            if (item.vt == VT_DISPATCH && item.pdispVal != nullptr)
            {
                INetFwRule* rule = nullptr;
                if (SUCCEEDED(item.pdispVal->QueryInterface(__uuidof(INetFwRule),
                                                           reinterpret_cast<void**>(&rule))) &&
                    rule != nullptr)
                {
                    if (ruleCovers(rule, exePath))
                    {
                        state = FirewallState::Allowed;
                    }
                    rule->Release();
                }
            }
            ::VariantClear(&item);
        }
        ::VariantClear(&item);

        iterator->Release();
        rules->Release();
        policy->Release();
        return state;
    }

    bool onPublicNetwork()
    {
        const ComScope com;
        if (!com.usable())
        {
            return false;
        }

        INetFwPolicy2* policy = nullptr;
        if (FAILED(::CoCreateInstance(__uuidof(NetFwPolicy2), nullptr, CLSCTX_INPROC_SERVER,
                                      __uuidof(INetFwPolicy2), reinterpret_cast<void**>(&policy))) ||
            policy == nullptr)
        {
            return false;
        }

        LONG profiles = 0;
        const bool ok = SUCCEEDED(policy->get_CurrentProfileTypes(&profiles));
        policy->Release();
        // A machine can be on several networks at once, so this is a mask
        // and not a value. Public appearing anywhere in it is enough: the
        // adapter the other player is reaching may well be that one.
        return ok && (profiles & NET_FW_PROFILE2_PUBLIC) != 0;
    }

    FirewallRequest allowInboundUdp()
    {
        if (inboundUdpState() == FirewallState::Allowed)
        {
            return FirewallRequest::AlreadyAllowed;
        }

        const std::wstring exePath = executablePathW();
        if (exePath.empty())
        {
            return FirewallRequest::Failed;
        }

        // netsh, not PowerShell: it is present on every edition including
        // Home, it is not subject to an execution policy, and it does not
        // flash a console window through a script host.
        //
        // Private and Domain only. Public is the profile Windows puts you on
        // in a cafe, and a game is not a reason to accept inbound traffic
        // there. `dir=in protocol=udp` with no port: the rule follows the
        // executable, so changing the port later does not ask again.
        std::wstring parameters =
            L"advfirewall firewall add rule name=\"";
        parameters += kRuleName;
        parameters += L"\" dir=in action=allow protocol=udp profile=private,domain program=\"";
        parameters += exePath;
        parameters += L"\" enable=yes";

        wchar_t systemDirectory[MAX_PATH]{};
        const UINT length = ::GetSystemDirectoryW(systemDirectory, MAX_PATH);
        std::wstring netsh =
            (length > 0 && length < MAX_PATH) ? std::wstring(systemDirectory) + L"\\netsh.exe"
                                              : std::wstring(L"netsh.exe");

        SHELLEXECUTEINFOW info{};
        info.cbSize = sizeof(info);
        // NOCLOSEPROCESS so the exit code can be read: "the prompt was
        // accepted" and "the rule was created" are different claims, and
        // only the second one is worth telling the player.
        info.fMask = SEE_MASK_NOCLOSEPROCESS | SEE_MASK_NOASYNC;
        info.lpVerb = L"runas"; // this, and only this, is what raises UAC
        info.lpFile = netsh.c_str();
        info.lpParameters = parameters.c_str();
        info.nShow = SW_HIDE;

        if (::ShellExecuteExW(&info) == FALSE)
        {
            const DWORD error = ::GetLastError();
            // 1223 is the user closing the UAC dialog. It is the expected
            // answer from someone who does not want this, and reporting it
            // as a failure would be a lie that invites them to try again.
            return (error == ERROR_CANCELLED) ? FirewallRequest::Declined
                                              : FirewallRequest::Failed;
        }
        if (info.hProcess == nullptr)
        {
            return FirewallRequest::Failed;
        }

        ::WaitForSingleObject(info.hProcess, 30000);
        DWORD exitCode = 1;
        ::GetExitCodeProcess(info.hProcess, &exitCode);
        ::CloseHandle(info.hProcess);

        if (exitCode != 0)
        {
            SW_LOG_ERROR("Firewall", "netsh refused the rule (exit code {})",
                         static_cast<u32>(exitCode));
            return FirewallRequest::Failed;
        }

        // Trust the observation, not the exit code: re-read the firewall and
        // report what is actually there now.
        return (inboundUdpState() == FirewallState::Allowed) ? FirewallRequest::Added
                                                             : FirewallRequest::Failed;
    }

#else // not Windows

    FirewallState inboundUdpState()
    {
        return FirewallState::Unknown;
    }

    bool onPublicNetwork()
    {
        return false;
    }

    FirewallRequest allowInboundUdp()
    {
        return FirewallRequest::Unsupported;
    }

#endif

    std::string describe(FirewallRequest result)
    {
        switch (result)
        {
            case FirewallRequest::AlreadyAllowed: return "firewall already allows it";
            case FirewallRequest::Added: return "firewall rule added";
            case FirewallRequest::Declined:
                return "firewall rule REFUSED - others may not be able to join";
            case FirewallRequest::Failed: return "could not add the firewall rule";
            case FirewallRequest::Unsupported: return "no firewall step needed on this platform";
        }
        return "unknown";
    }
} // namespace sw::platform
